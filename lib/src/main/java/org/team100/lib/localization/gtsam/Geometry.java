package org.team100.lib.localization.gtsam;

import org.team100.lib.camera.Distortion;
import org.team100.lib.camera.Intrinsic;
import org.team100.lib.util.Math100;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Nat;

import gtsam.Cal3DS2;
import gtsam.Matrix3;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose2;
import gtsam.Pose3;
import gtsam.Rot3;
import gtsam.Vector3;

public class Geometry {
    public static Pose2 toPose2(Pose2d p) {
        return new Pose2(p.getX(), p.getY(), p.getRotation().getRadians());
    }

    public static Pose2d toPose2d(Pose2 p) {
        return new Pose2d(p.x(), p.y(), new Rotation2d(p.theta()));
    }

    public static Twist2d twistFromVector(Vector3 v) {
        return new Twist2d(v.at(0), v.at(1), v.at(2));
    }

    public static Point3 toPoint3(Translation3d t) {
        return new Point3(t.getX(), t.getY(), t.getZ());
    }

    public static Translation3d toTranslation3d(Point3 t) {
        return new Translation3d(t.x(), t.y(), t.z());
    }

    public static Point2 toPoint2(Translation2d t) {
        return new Point2(t.getX(), t.getY());
    }

    public static Pose3 toPose3(Transform3d t) {
        return new Pose3(toRot3(t.getRotation()), toPoint3(t.getTranslation()));
    }

    public static Pose3d toPose3d(Pose3 p) {
        Translation3d t = toTranslation3d(p.translation());
        Rotation3d r = toRotation3d(p.rotation());
        return new Pose3d(t, r);
    }

    public static Rot3 toRot3(Rotation3d r) {
        Matrix<N3, N3> m = r.toMatrix();
        return new Rot3(
                m.get(0, 0), m.get(0, 1), m.get(0, 2), //
                m.get(1, 0), m.get(1, 1), m.get(1, 2), //
                m.get(2, 0), m.get(2, 1), m.get(2, 2));
    }

    public static Rotation3d toRotation3d(Rot3 r) {
        Matrix3 m = r.matrix();
        Matrix<N3, N3> R = MatBuilder.fill(Nat.N3(), Nat.N3(), //
                m.at(0, 0), m.at(0, 1), m.at(0, 2), //
                m.at(1, 0), m.at(1, 1), m.at(1, 2), //
                m.at(2, 0), m.at(2, 1), m.at(2, 2));
        return new Rotation3d(R);
    }

    public static Cal3DS2 toCal3DS2(Intrinsic i, Distortion d) {
        return new Cal3DS2(
                i.fx(), i.fy(), 0, i.cx(), i.cy(), d.k1(), d.k2(), d.p1(), d.p2());
    }

    public static Pose2 interpolate(Pose2 a, Pose2 b, double s) {
        // TODO: interpolate with a twist
        return new Pose2(
                Math100.interpolate(a.x(), b.x(), s),
                Math100.interpolate(a.y(), b.y(), s),
                Math100.interpolate(a.theta(), b.theta(), s));

    }

}
