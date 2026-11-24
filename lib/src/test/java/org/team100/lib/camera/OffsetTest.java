package org.team100.lib.camera;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
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
import gtsam.NonlinearFactorGraph;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose3;
import gtsam.PriorFactor;
import gtsam.Rot3;
import gtsam.Values;
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
            // many tags at low-ish resolution, do not decimate
            conf.quadDecimate = 1.0f;
            detector.setConfig(conf);

            Mat image = ImgUtil.loadImage(getClass(), "onetag.png");
            Mat timage = ImgUtil.transform(ImgUtil.t(600, 600, 6000), image, 1500);
            Imgcodecs.imwrite("/tmp/transformedimg.png", timage);

            Key pose0 = Key.X(0);
            Rot3 priorRot = new Rot3();
            Point3 priorTranslation = new Point3(0, 0, -1);
            Pose3 pose_prior = new Pose3(priorRot, priorTranslation);
            shared_ptr<Isotropic> prior_noise = Isotropic.Sigma(6, 10.0);

            shared_ptr<Cal3DS2> K = Cal3DS2.shared(
                    1500, 1500,
                    0, timage.size().width / 2, timage.size().height / 2,
                    0, 0, 0, 0, 0);
            shared_ptr<Isotropic> px_noise = Isotropic.Sigma(2, 1.0);

            Values values = new Values();
            values.insert(pose0, pose_prior);

            NonlinearFactorGraph graph = new NonlinearFactorGraph();
            graph.add(PriorFactor.PriorFactorPose3(pose0, pose_prior, prior_noise));
            AprilTagDetection[] detections = detector.detect(timage);
            // there's only one
            assertEquals(1, detections.length);
            AprilTagDetection r = detections[0];
            assertEquals(0, r.getHamming());

            Translation3d[] landmarks = landmarks();
            double[] corners = r.getCorners();

            graph.add(GenericProjectionFactor3.GenericProjectionFactor3Cal3DS2(
                    pose0, new Point3(
                            landmarks[0].getX(),
                            landmarks[0].getY(),
                            landmarks[0].getZ()),
                    new Point2(corners[0], corners[1]),
                    K, px_noise, false));
            graph.add(GenericProjectionFactor3.GenericProjectionFactor3Cal3DS2(
                    pose0, new Point3(
                            landmarks[1].getX(),
                            landmarks[1].getY(),
                            landmarks[1].getZ()),
                    new Point2(corners[2], corners[3]),
                    K, px_noise, false));
            graph.add(GenericProjectionFactor3.GenericProjectionFactor3Cal3DS2(
                    pose0, new Point3(
                            landmarks[2].getX(),
                            landmarks[2].getY(),
                            landmarks[2].getZ()),
                    new Point2(corners[4], corners[5]),
                    K, px_noise, false));
            graph.add(GenericProjectionFactor3.GenericProjectionFactor3Cal3DS2(
                    pose0, new Point3(
                            landmarks[3].getX(),
                            landmarks[3].getY(),
                            landmarks[3].getZ()),
                    new Point2(corners[6], corners[7]),
                    K, px_noise, false));



        } catch (

        Exception e) {
            e.printStackTrace();
            fail();
        }

    }

    Translation3d[] landmarks() {
        Pose3d center = new Pose3d();
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

}
