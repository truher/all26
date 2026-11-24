package org.team100.lib.camera;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.team100.lib.localization.gtsam.Geometry;
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
import gtsam.GenericProjectionFactor3;
import gtsam.Key;
import gtsam.LevenbergMarquardtOptimizer;
import gtsam.LevenbergMarquardtParams;
import gtsam.Marginals;
import gtsam.NonlinearFactorGraph;
import gtsam.PinholeCamera;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose3;
import gtsam.PriorFactor;
import gtsam.Rot3;
import gtsam.Values;
import gtsam.Vector;
import gtsam.shared_ptr;
import gtsam.noiseModel.Isotropic;

/**
 * Exercise the camera-offset measurement path, using GenericProjectinFactor3,
 * with fixed landmarks, fixed calibration, and variable pose.
 */
public class OffsetTest {
    static {
        try {
            OpenCvLoader.forceLoad();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Test
    void test0() {
        try (AprilTagDetector detector = new AprilTagDetector()) {
            detector.addFamily("tag36h11");
            Config conf = new AprilTagDetector.Config();
            conf.numThreads = 2;
            conf.quadDecimate = 1.0f;
            detector.setConfig(conf);

            Mat image = ImgUtil.loadImage(getClass(), "onetag.png");

            // Shrink and shift the image to roughly 25%, so the tag length is about 20% of
            // the frame width.
            Mat timage = ImgUtil.transform(ImgUtil.t(600, 600, 6000), image, 1500);
            Imgcodecs.imwrite("/tmp/transformedimg.png", timage);

            Key pose0 = Key.X(0);
            Rot3 priorRot = new Rot3();
            Point3 priorTranslation = new Point3(0, 0, -1);
            Pose3 pose_prior = new Pose3(priorRot, priorTranslation);
            // The prior is essentially meaningless.
            shared_ptr<Isotropic> prior_noise = Isotropic.Sigma(6, 100.0);

            // choose f to make the tag appear about 1 m away.
            shared_ptr<Cal3DS2> K = Cal3DS2.shared(
                    900, 900,
                    0, timage.size().width / 2, timage.size().height / 2,
                    0, 0, 0, 0, 0);
            // Note the sigma here is less than one pixel, since the observed
            // median reprojection error is, indeed, about 0.5.
            // This has no effect on the solution (since there is just one kind
            // of factor and no actual noise), it just affects the estimated sigma.
            shared_ptr<Isotropic> px_noise = Isotropic.Sigma(2, 0.5);

            Values values = new Values();
            values.insert(pose0, pose_prior);

            NonlinearFactorGraph graph = new NonlinearFactorGraph();
            graph.add(PriorFactor.PriorFactorPose3(pose0, pose_prior, prior_noise));
            AprilTagDetection[] detections = detector.detect(timage);

            // There's only one tag in the image.
            assertEquals(1, detections.length);
            AprilTagDetection r = detections[0];

            // It should be easy to find.
            assertEquals(0, r.getHamming());

            // Add a projection factor for each corner.
            Translation3d[] landmarks = landmarks();
            double[] corners = r.getCorners();

            // Take one second of input.
            // The repeated frames make the pose estimate more confident,
            // but don't change the mean. If there were gaussian noise in the
            // pixel estimates, repeated frames would make the pose estimate
            // more confident. I've mostly not seen that in the real world: if
            // the camera is not moving, the pixel estimates don't change *at all*.
            // They might not be *correct* (i.e. the camera calibration is not
            // perfect).
            for (int i = 0; i < 30; ++i) {
                addFrame(pose0, K, px_noise, graph, landmarks, corners);
            }

            // This is an easy problem to solve, use the default params.
            LevenbergMarquardtParams params = new LevenbergMarquardtParams();
            params.setCeresDefaults();
            LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer(
                    graph, values, params);
            Values result = optimizer.optimize();

            // Print all results.
            result.print("result");

            // Print the estimated (mean) pose.
            Pose3 result_pose0 = result.atPose3(pose0);
            Pose3d rp0 = Geometry.toPose3d(result_pose0);
            System.out.printf("pose0 %s\n", rp0);

            // Print the estimated pose covariance.
            Marginals marginals = Marginals.QR(graph, result);
            Vector sigma = marginals.marginalCovariance(pose0).diagonal_cwiseSqrt();
            sigma.print("pose0");

            // Compute the pose estimate in WPI rotations (NWU, different from Rot3).
            Point3 t0 = result_pose0.translation();
            Translation3d t3d = Geometry.toTranslation3d(t0);
            Rot3 r0 = result_pose0.rotation();
            Rotation3d r3d = Geometry.toRotation3d(r0);

            // Print the estimated pose mean and std dev.
            // Our pose estimate std dev target is about 5 mm and 0.5 degrees.
            System.out.println();
            System.out.printf(
                    " mean translation (%6.3f, %6.3f, %6.3f) rotation (%6.3f, %6.3f, %6.3f)\n",
                    t3d.getX(), t3d.getY(), t3d.getZ(),
                    r3d.getX(), r3d.getY(), r3d.getZ());
            System.out.printf("sigma translation (%6.3f, %6.3f, %6.3f) rotation (%6.3f, %6.3f, %6.3f)\n",
                    sigma.at(3), sigma.at(4), sigma.at(5), sigma.at(0), sigma.at(1), sigma.at(2));
            System.out.println();

            // Compute and print the reprojection errors.
            PinholeCamera<Cal3DS2> camera0 = PinholeCamera.PinholeCameraCal3DS2(result_pose0, K.get());
            for (int i = 0; i < 4; ++i) {
                Translation3d li = landmarks[i];
                Point3 landmark = new Point3(li.getX(), li.getY(), li.getZ());
                Point2 estimate = camera0.project(landmark);
                Point2 measured = new Point2(corners[2 * i], corners[2 * i + 1]);
                double error = measured.minus(estimate).norm();
                System.out.printf(
                        "landmark (%8.3f, %8.3f, %8.3f) measurement (%8.3f, %8.3f) estimate (%8.3f, %8.3f) error %8.3f\n",
                        landmark.x(), landmark.y(), landmark.z(),
                        measured.x(), measured.y(),
                        estimate.x(), estimate.y(),
                        error);
            }

        } catch (

        Exception e) {
            e.printStackTrace();
            fail();
        }

    }

    // when the camera is still, each frame really is the same.
    private void addFrame(
            Key pose0,
            shared_ptr<Cal3DS2> K,
            shared_ptr<Isotropic> px_noise,
            NonlinearFactorGraph graph,
            Translation3d[] landmarks,
            double[] corners) {
        Translation3d l0 = landmarks[0];
        graph.add(GenericProjectionFactor3.GenericProjectionFactor3Cal3DS2(
                pose0, new Point3(l0.getX(), l0.getY(), l0.getZ()),
                new Point2(corners[0], corners[1]),
                K, px_noise, false));
        Translation3d l1 = landmarks[1];
        graph.add(GenericProjectionFactor3.GenericProjectionFactor3Cal3DS2(
                pose0, new Point3(l1.getX(), l1.getY(), l1.getZ()),
                new Point2(corners[2], corners[3]),
                K, px_noise, false));
        Translation3d l2 = landmarks[2];
        graph.add(GenericProjectionFactor3.GenericProjectionFactor3Cal3DS2(
                pose0, new Point3(l2.getX(), l2.getY(), l2.getZ()),
                new Point2(corners[4], corners[5]),
                K, px_noise, false));
        Translation3d l3 = landmarks[3];
        graph.add(GenericProjectionFactor3.GenericProjectionFactor3Cal3DS2(
                pose0, new Point3(l3.getX(), l3.getY(), l3.getZ()),
                new Point2(corners[6], corners[7]),
                K, px_noise, false));
    }

    Translation3d[] landmarks() {
        Pose3d center = new Pose3d();
        // this is the normal FRC tag size, 16.51 cm.
        double marker_length = 0.1651;
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

}
