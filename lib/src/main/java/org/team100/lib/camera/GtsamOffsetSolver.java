package org.team100.lib.camera;

import java.util.List;

import org.team100.lib.localization.gtsam.Geometry;
import org.team100.lib.localization.network.BlipWithCorners;
import org.team100.lib.uncertainty.NoisyTransform3d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

import gtsam.Cal3DS2;
import gtsam.GenericProjectionFactor3;
import gtsam.Key;
import gtsam.LevenbergMarquardtOptimizer;
import gtsam.LevenbergMarquardtParams;
import gtsam.Marginals;
import gtsam.NonlinearFactorGraph;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose3;
import gtsam.PriorFactor;
import gtsam.Rot3;
import gtsam.Values;
import gtsam.Vector;
import gtsam.shared_ptr;
import gtsam.noiseModel.Isotropic;

public class GtsamOffsetSolver {

    private final shared_ptr<Isotropic> m_pixelNoise;
    private final shared_ptr<Isotropic> m_posePriorNoise;
    private final Translation3d[] m_landmarks;

    public GtsamOffsetSolver() {
        m_pixelNoise = Isotropic.Sigma(2, 0.5);
        m_posePriorNoise = Isotropic.Sigma(6, 100.0);
        m_landmarks = landmarks();
    }

    /**
     * Return a camera-relative ("z forward") transform for the given blips.
     * 
     * @param blips list of sightings of the same tag by the same camera.
     */
    public NoisyTransform3d solve(Camera camera, List<BlipWithCorners> blips) {
        Key pose = Key.X(0);
        Pose3 pose_prior = new Pose3(new Rot3(), new Point3(0, 0, -1));

        Values values = new Values();
        values.insert(pose, pose_prior);

        NonlinearFactorGraph graph = new NonlinearFactorGraph();
        graph.add(PriorFactor.PriorFactorPose3(pose, pose_prior, m_posePriorNoise));

        Intrinsic intrinsic = Intrinsic.get(camera);
        Distortion distortion = Distortion.get(camera);

        shared_ptr<Cal3DS2> K = Cal3DS2.shared(
                intrinsic.fx(),
                intrinsic.fy(),
                0,
                intrinsic.cx(),
                intrinsic.cy(),
                distortion.k1(),
                distortion.k2(),
                distortion.p1(),
                distortion.p2(),
                1e-5);

        for (BlipWithCorners blip : blips) {
            addFrame(pose, K, graph, blip.getCorners());
        }

        LevenbergMarquardtParams params = new LevenbergMarquardtParams();
        params.setCeresDefaults();
        LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer(
                graph, values, params);
        Values result = optimizer.optimize();

        Pose3 resultPose = result.atPose3(pose);
        // Pose of the camera relative to the tag at the origin.
        Pose3d resultPose3d = Geometry.toPose3d(resultPose);
        // Transform from camera to the origin.
        Transform3d transform = new Transform3d(resultPose3d, new Pose3d());

        // Standard deviation of the camera pose.
        Marginals marginals = Marginals.QR(graph, result);
        Vector sigmaV = marginals.marginalCovariance(pose).diagonal_cwiseSqrt();
        double[] sigma = new double[sigmaV.rows()];
        for (int i = 0; i < sigmaV.rows(); ++i) {
            sigma[i] = sigmaV.at(i);
        }

        // TODO: compute and log reprojection error (see OffsetTest.java)
        return new NoisyTransform3d(transform, sigma);
    }

    void addFrame(
            Key key,
            shared_ptr<Cal3DS2> K,
            NonlinearFactorGraph graph,
            float[] corners) {
        Translation3d l0 = m_landmarks[0];
        graph.add(GenericProjectionFactor3.GenericProjectionFactor3Cal3DS2(
                key, new Point3(l0.getX(), l0.getY(), l0.getZ()),
                new Point2(corners[0], corners[1]),
                K, m_pixelNoise, false));
        Translation3d l1 = m_landmarks[1];
        graph.add(GenericProjectionFactor3.GenericProjectionFactor3Cal3DS2(
                key, new Point3(l1.getX(), l1.getY(), l1.getZ()),
                new Point2(corners[2], corners[3]),
                K, m_pixelNoise, false));
        Translation3d l2 = m_landmarks[2];
        graph.add(GenericProjectionFactor3.GenericProjectionFactor3Cal3DS2(
                key, new Point3(l2.getX(), l2.getY(), l2.getZ()),
                new Point2(corners[4], corners[5]),
                K, m_pixelNoise, false));
        Translation3d l3 = m_landmarks[3];
        graph.add(GenericProjectionFactor3.GenericProjectionFactor3Cal3DS2(
                key, new Point3(l3.getX(), l3.getY(), l3.getZ()),
                new Point2(corners[6], corners[7]),
                K, m_pixelNoise, false));
    }

    static Translation3d[] landmarks() {
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
