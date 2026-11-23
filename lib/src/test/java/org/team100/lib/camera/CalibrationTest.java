package org.team100.lib.camera;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.net.http.HttpClient.Version;

import org.junit.jupiter.api.Test;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.team100.lib.localization.gtsam.Geometry;
import org.team100.lib.testing.TestUtil;
import org.team100.lib.util.CDF;
import org.team100.lib.util.ImgUtil;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.vision.apriltag.AprilTagDetection;
import org.wpilib.vision.apriltag.AprilTagDetector;
import org.wpilib.vision.apriltag.AprilTagDetector.Config;
import org.wpilib.vision.camera.OpenCvLoader;

import gtsam.Cal3DS2;
import gtsam.GeneralSFMFactor3;
import gtsam.Key;
import gtsam.LevenbergMarquardtOptimizer;
import gtsam.LevenbergMarquardtParams;
import gtsam.Matrix;
import gtsam.Matrix3;
import gtsam.NonlinearFactorGraph;
import gtsam.PinholeCamera;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose3;
import gtsam.PriorFactor;
import gtsam.Rot3;
import gtsam.Values;
import gtsam.Vector;
import gtsam.Vector2;
import gtsam.Vector4;
import gtsam.shared_ptr;
import gtsam.NonlinearOptimizerParams.Verbosity;
import gtsam.noiseModel.Diagonal;
import gtsam.noiseModel.Isotropic;

public class CalibrationTest {
    static {
        try {
            OpenCvLoader.forceLoad();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * One head-on shot is ambiguous, but it comes up with *something*.
     */
    @Test
    void test0() throws IOException {
        // we're just using the java detector for convenience
        try (AprilTagDetector detector = new AprilTagDetector()) {
            detector.addFamily("tag36h11");
            Config conf = new AprilTagDetector.Config();
            conf.numThreads = 2;
            // many tags at low-ish resolution, do not decimate
            conf.quadDecimate = 1.0f;
            detector.setConfig(conf);

            Mat image = loadImage("tangram.png");
            System.out.println(image);

            Key pose_key = Key.X(0);
            // we know kinda where the camera is pointing
            Rot3 priorRot = new Rot3(//
                    0, 0, 1, //
                    -1, 0, 0, //
                    0, -1, 0);
            Point3 priorTranslation = new Point3(0, 0, -100);
            Pose3 pose_prior = new Pose3(priorRot, priorTranslation);
            shared_ptr<Isotropic> prior_noise = Isotropic.Sigma(6, 1.0);

            Key calib_key = Key.K(0);
            Cal3DS2 cal_prior = new Cal3DS2(200, 200, 0, 100, 100, 0, 0);
            shared_ptr<Isotropic> cal_noise = Isotropic.Sigma(9, 100);

            // pixel noise
            shared_ptr<Isotropic> sfm_noise = Isotropic.Sigma(2, 1.0);

            Values values = new Values();
            values.insert(pose_key, pose_prior);
            values.insert(calib_key, cal_prior);

            NonlinearFactorGraph graph = new NonlinearFactorGraph();
            graph.add(PriorFactor.PriorFactorPose3(pose_key, pose_prior, prior_noise));
            graph.add(PriorFactor.PriorFactorCal3DS2(calib_key, cal_prior, cal_noise));

            for (AprilTagDetection r : detector.detect(image)) {
                System.out.println(r.getId());
                // System.out.println(Arrays.toString(r.getCorners()));
                assertEquals("tag36h11", r.getFamily());
                // assertEquals(1, r.getId());
                assertEquals(0, r.getHamming());
                double[] corners = r.getCorners();

                int id = r.getId();
                Translation3d[] landmarks = landmarks(id);
                // System.out.println(Arrays.toString(landmarks));

                addFactor(id, pose_key, calib_key, sfm_noise, graph, landmarks, corners, 0);
                addFactor(id, pose_key, calib_key, sfm_noise, graph, landmarks, corners, 1);
                addFactor(id, pose_key, calib_key, sfm_noise, graph, landmarks, corners, 2);
                addFactor(id, pose_key, calib_key, sfm_noise, graph, landmarks, corners, 3);
            }

            LevenbergMarquardtParams params = new LevenbergMarquardtParams();
            LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer(
                    graph, values, params);
            Values result = optimizer.optimize();
            result.print("result");

        } catch (Exception e) {
            e.printStackTrace();
            fail();
        }
    }

    /**
     * Adjust the image so that it can be solved.
     */
    @Test
    void test1() throws IOException {
        // we're just using the java detector for convenience
        try (AprilTagDetector detector = new AprilTagDetector()) {
            detector.addFamily("tag36h11");
            Config conf = new AprilTagDetector.Config();
            conf.numThreads = 2;
            // many tags at low-ish resolution, do not decimate
            conf.quadDecimate = 1.0f;
            detector.setConfig(conf);

            Mat image = loadImage("tangram.png");
            // System.out.println(image);

            Key pose0 = Key.X(0);
            Key pose1 = Key.X(1);
            Key pose2 = Key.X(2);
            Rot3 priorRot = new Rot3();
            Point3 priorTranslation = new Point3(0, 0, -1);
            Pose3 pose_prior = new Pose3(priorRot, priorTranslation);
            shared_ptr<Isotropic> prior_noise = Isotropic.Sigma(6, 10.0);

            double f = 1500;

            Key calib_key = Key.K(0);
            Cal3DS2 cal_prior = new Cal3DS2(
                    f, f, 0, image.size().width / 2, image.size().height / 2, 0, 0);
            // focal length is pretty free, center is less free, skew is not very free.
            shared_ptr<Diagonal> cal_noise = Diagonal
                    .Sigmas(new Vector(new double[] { 100, 100, 0.1, 10, 10, 0.1, 0.1, 0.1, 0.1 }));

            // pixel noise
            shared_ptr<Isotropic> sfm_noise = Isotropic.Sigma(2, 1.0);

            Values values = new Values();
            values.insert(pose0, pose_prior);
            values.insert(pose1, pose_prior);
            values.insert(pose2, pose_prior);
            values.insert(calib_key, cal_prior);

            NonlinearFactorGraph graph = new NonlinearFactorGraph();
            graph.add(PriorFactor.PriorFactorPose3(pose0, pose_prior, prior_noise));
            graph.add(PriorFactor.PriorFactorPose3(pose1, pose_prior, prior_noise));
            graph.add(PriorFactor.PriorFactorPose3(pose2, pose_prior, prior_noise));
            graph.add(PriorFactor.PriorFactorCal3DS2(calib_key, cal_prior, cal_noise));

            // head-on image detections
            addFactors(detector, image, pose0, calib_key, sfm_noise, graph);

            // add rotated image detections
            // Mat rotated_image = ImgUtil.ro@t(ImgUtil.yrot(0.5), image);
            Mat rotated_image = ImgUtil.rot(ImgUtil.xrot(0.5, f), image, f);
            Imgcodecs.imwrite("/tmp/img.png", rotated_image);
            addFactors(detector, rotated_image, pose1, calib_key, sfm_noise, graph);

            Mat rotated_image2 = ImgUtil.rot(ImgUtil.yrot(0.5, f), image, f);
            Imgcodecs.imwrite("/tmp/img2.png", rotated_image2);
            addFactors(detector, rotated_image2, pose2, calib_key, sfm_noise, graph);

            LevenbergMarquardtParams params = new LevenbergMarquardtParams();
            params.setCeresDefaults();
            // this particular problem wants about 80 iterations, so the default, 50, is not enough.
            params.setMaxIterations(100);
            // turn all the tolerances off to see what happens
            params.setRelativeErrorTol(0);
            params.setErrorTol(0);
            // watch the error
            params.setVerbosity(Verbosity.ERROR);
            LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer(
                    graph, values, params);
            Values result = optimizer.optimize();
            // about 80 iterations
            System.out.printf("iterations: %d\n", optimizer.iterations());
            result.print("result");

            Cal3DS2 result_cal = result.atCal3DS2(calib_key);
            Matrix3 K = result_cal.K();
            System.out.printf("K fx %f fy %f cx %f cy %f\n",
                    K.at(0, 0), K.at(1, 1), K.at(0, 2), K.at(1, 2));
            Vector4 k = result_cal.k();
            System.out.printf("k [%f %f %f %f]\n",
                    k.at(0), k.at(1), k.at(2), k.at(3));

            Pose3 result_pose0 = result.atPose3(pose0);
            Pose3d rp0 = Geometry.toPose3d(result_pose0);
            System.out.printf("pose0 %s\n", rp0);
            Pose3 result_pose1 = result.atPose3(pose1);
            Pose3d rp1 = Geometry.toPose3d(result_pose1);
            System.out.printf("pose1 %s\n", rp1);
            Pose3 result_pose2 = result.atPose3(pose2);
            Pose3d rp2 = Geometry.toPose3d(result_pose2);
            System.out.printf("pose2 %s\n", rp2);

            // Marginals marginals = Marginals.QR(graph, result);
            // marginals.marginalCovariance(pose0).diagonal_cwiseSqrt().print("pose0");
            // marginals.marginalCovariance(pose1).diagonal_cwiseSqrt().print("pose1");
            // marginals.marginalCovariance(calib_key).diagonal_cwiseSqrt().print("calib");

            // graph.printErrors(result, "errors");

            // reprojection
            CDF cdf = new CDF();
            Point3 t0 = result_pose0.translation();
            PinholeCamera<Cal3DS2> camera0 = PinholeCamera.PinholeCameraCal3DS2(result_pose0, result_cal);
            for (AprilTagDetection r : detector.detect(image)) {
                int id = r.getId();
                Translation3d[] landmarks = landmarks(id);
                double[] corners = r.getCorners();
                for (int i = 0; i < 4; ++i) {
                    Translation3d li = landmarks[i];
                    Point3 landmark = new Point3(li.getX(), li.getY(), li.getZ());
                    Point2 estimate = camera0.project(landmark);
                    Point2 measured = new Point2(corners[2 * i], corners[2 * i + 1]);
                    double error = measured.minus(estimate).norm();
                    cdf.add(error);
                    System.out.printf(
                            "translation (%8.3f, %8.3f, %8.3f) landmark (%8.3f, %8.3f, %8.3f) measurement (%8.3f, %8.3f) estimate (%8.3f, %8.3f) error %8.3f\n",
                            t0.x(), t0.y(), t0.z(),
                            landmark.x(), landmark.y(), landmark.z(),
                            measured.x(), measured.y(),
                            estimate.x(), estimate.y(),
                            error);
                }
            }
            Point3 t1 = result_pose1.translation();
            PinholeCamera<Cal3DS2> camera1 = PinholeCamera.PinholeCameraCal3DS2(result_pose1, result_cal);
            for (AprilTagDetection r : detector.detect(rotated_image)) {
                int id = r.getId();
                Translation3d[] landmarks = landmarks(id);
                double[] corners = r.getCorners();
                for (int i = 0; i < 4; ++i) {
                    Translation3d li = landmarks[i];
                    Point3 landmark = new Point3(li.getX(), li.getY(), li.getZ());
                    Point2 estimate = camera1.project(landmark);
                    Point2 measured = new Point2(corners[2 * i], corners[2 * i + 1]);
                    double error = measured.minus(estimate).norm();
                    cdf.add(error);
                    System.out.printf(
                            "translation (%8.3f, %8.3f, %8.3f) landmark (%8.3f, %8.3f, %8.3f) measurement (%8.3f, %8.3f) estimate (%8.3f, %8.3f) error %8.3f\n",
                            t1.x(), t1.y(), t1.z(),
                            landmark.x(), landmark.y(), landmark.z(),
                            measured.x(), measured.y(),
                            estimate.x(), estimate.y(),
                            error);
                }
            }
            Point3 t2 = result_pose2.translation();
            PinholeCamera<Cal3DS2> camera2 = PinholeCamera.PinholeCameraCal3DS2(result_pose2, result_cal);
            for (AprilTagDetection r : detector.detect(rotated_image2)) {
                int id = r.getId();
                Translation3d[] landmarks = landmarks(id);
                double[] corners = r.getCorners();
                for (int i = 0; i < 4; ++i) {
                    Translation3d li = landmarks[i];
                    Point3 landmark = new Point3(li.getX(), li.getY(), li.getZ());
                    Point2 estimate = camera2.project(landmark);
                    Point2 measured = new Point2(corners[2 * i], corners[2 * i + 1]);
                    double error = measured.minus(estimate).norm();
                    cdf.add(error);
                    System.out.printf(
                            "translation (%8.3f, %8.3f, %8.3f) landmark (%8.3f, %8.3f, %8.3f) measurement (%8.3f, %8.3f) estimate (%8.3f, %8.3f) error %8.3f\n",
                            t2.x(), t2.y(), t2.z(),
                            landmark.x(), landmark.y(), landmark.z(),
                            measured.x(), measured.y(),
                            estimate.x(), estimate.y(),
                            error);
                }
            }

            System.out.println("ERROR CDF");
            for (double e = 0; e < 1; e += 0.025) {
                System.out.printf("%f, %f\n", e, cdf.get(e));
            }

        } catch (Exception e) {
            e.printStackTrace();
            fail();
        }
    }

    private void addFactors(AprilTagDetector detector, Mat image, Key pose, Key calib_key,
            shared_ptr<Isotropic> sfm_noise, NonlinearFactorGraph graph) {
        for (AprilTagDetection r : detector.detect(image)) {
            if (r.getHamming() > 0)
                continue;
            int id = r.getId();
            Translation3d[] landmarks = landmarks(id);
            // System.out.println(Arrays.toString(landmarks));
            double[] corners = r.getCorners();
            addFactor(id, pose, calib_key, sfm_noise, graph, landmarks, corners, 0);
            addFactor(id, pose, calib_key, sfm_noise, graph, landmarks, corners, 1);
            addFactor(id, pose, calib_key, sfm_noise, graph, landmarks, corners, 2);
            addFactor(id, pose, calib_key, sfm_noise, graph, landmarks, corners, 3);
        }
    }

    private void addFactor(
            int id,
            Key pose_key,
            Key calib_key,
            shared_ptr<Isotropic> sfm_noise,
            NonlinearFactorGraph graph,
            Translation3d[] landmarks,
            double[] corners,
            int i) {
        Translation3d li = landmarks[i];
        Point3 landmark = new Point3(li.getX(), li.getY(), li.getZ());
        Point2 measured = new Point2(corners[2 * i], corners[2 * i + 1]);
        System.out.printf("ID: %5d LANDMARK: %6.3f %6.3f %6.3f MEASURED %6.3f %6.3f\n",
                id, landmark.x(), landmark.y(), landmark.z(), measured.x(), measured.y());
        graph.add(GeneralSFMFactor3.GeneralSFMFactor3Cal3DS2(
                landmark,
                measured,
                sfm_noise,
                pose_key,
                calib_key,
                false));
    }

    /**
     * Verify that the SFM thing works at all.
     */
    @Test
    void test2() {
        Pose3 camera_pose = new Pose3();
        Key pose_key = Key.X(0);

        Point3 landmark = new Point3(0, 0, 5); // Point 5m in front of camera

        Point2 measured_pt = new Point2(320, 240);// Measurement in pixels
        shared_ptr<Isotropic> sfm_noise = Isotropic.Sigma(2, 1.0);
        Key calib_key = Key.K(0);

        shared_ptr<GeneralSFMFactor3<Cal3DS2>> sfmFactor = GeneralSFMFactor3.GeneralSFMFactor3Cal3DS2(
                landmark, measured_pt, sfm_noise, pose_key, calib_key, false);

        // should print
        // keys = { x0 l0 k0 }
        // noise model: unit (2)
        // GeneralSFMFactor2: .z[
        // 320;
        // 240
        // ]
        sfmFactor.get().print();

        Values values2 = new Values();
        Cal3DS2 calibration = new Cal3DS2(500, 500, 0, 320, 240, 0, 0, 0, 0); // fx, fy, s, u0, v0
        values2.insert(pose_key, camera_pose);
        values2.insert(calib_key, calibration);

        // should be zero
        double error2 = sfmFactor.get().error(values2);
        System.out.printf("Error for GeneralSFMFactor2: %f\n", error2);

        Vector2 errV = sfmFactor.get().evaluateError(
                camera_pose, calibration, new Matrix(), new Matrix());

        // should be (0, 0)
        System.out.printf("errV %f %f\n", errV.at(0), errV.at(1));
    }

    /**
     * the tag array starts at the lower left, scans to the right,
     * then up, using "x forward" so the "x" is always zero.
     */
    @Test
    void testCenters() {
        // spacing is 0.0635*1.25 = 0.0793
        TestUtil.verify(new Translation3d(0, 0, 0), center(0).getTranslation());
        TestUtil.verify(new Translation3d(0.079, 0, 0), center(1).getTranslation());
        TestUtil.verify(new Translation3d(0, -0.079, 0), center(10).getTranslation());
        for (int i = 0; i < 100; ++i) {
            Pose3d c = center(i);
            System.out.printf("%3d %6.3f %6.3f %6.3f\n", i, c.getX(), c.getY(), c.getZ());
        }
    }

    /**
     * Center pose of the given tag id.
     * 
     * In the Tangram image, zero is lower left, one is to the right of that,
     * ten is above zero, etc.
     * 
     * This uses camear coordinates, x-right, y-down, z-forward.
     */
    Pose3d center(int tagId) {
        double marker_length = 0.0635;
        int marker_grid_width = 10;
        double tag_spacing = 0.25;
        int col = tagId % marker_grid_width;
        int row = tagId / marker_grid_width;
        double grid_spacing = marker_length * (1 + tag_spacing);
        double x = col * grid_spacing;
        double y = -1.0 * row * grid_spacing;
        return new Pose3d(x, y, 0, Rotation3d.ZERO);
    }

    /**
     * landmarks using a more-natural camera coordinate system,
     * x-right, y-down, z-forward.
     */
    Translation3d[] landmarks(int tagId) {
        Pose3d center = center(tagId);
        double marker_length = 0.0635;
        double H = marker_length / 2;
        // these are in x-forward, y-left, z-up coordinates.
        Transform3d LL = new Transform3d(-H, H, 0, Rotation3d.ZERO);
        Transform3d LR = new Transform3d(H, H, 0, Rotation3d.ZERO);
        Transform3d UR = new Transform3d(H, -H, 0, Rotation3d.ZERO);
        Transform3d UL = new Transform3d(-H, -H, 0, Rotation3d.ZERO);
        return new Translation3d[] {
                center.transformBy(LL).getTranslation(),
                center.transformBy(LR).getTranslation(),
                center.transformBy(UR).getTranslation(),
                center.transformBy(UL).getTranslation() };
    }

    // from wpilib
    public Mat loadImage(String resource) {
        URL res = getClass().getResource(resource);
        if (res == null)
            throw new IllegalArgumentException(String.format("no file at %s", resource));
        try (InputStream is = res.openStream()) {
            try (ByteArrayOutputStream os = new ByteArrayOutputStream(is.available())) {
                byte[] buffer = new byte[4096];
                int bytesRead;
                while ((bytesRead = is.read(buffer)) != -1) {
                    os.write(buffer, 0, bytesRead);
                }
                Mat encoded = new Mat(1, os.size(), CvType.CV_8U);
                encoded.put(0, 0, os.toByteArray());
                Mat image = Imgcodecs.imdecode(encoded, Imgcodecs.IMREAD_COLOR);
                encoded.release();
                Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2GRAY);
                return image;
            }
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }

    }

}
