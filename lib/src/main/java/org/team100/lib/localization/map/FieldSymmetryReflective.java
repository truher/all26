package org.team100.lib.localization.map;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class FieldSymmetryReflective implements FieldSymmetry {
    private final double m_length;
    private final Supplier<Optional<Alliance>> m_alliance;

    public FieldSymmetryReflective(
            double length,
            Supplier<Optional<Alliance>> alliance) {
        m_length = length;
        m_alliance = alliance;
    }

    private boolean blue() {
        return m_alliance.get().orElse(Alliance.BLUE) == Alliance.BLUE;
    }

    /** Inverts x */
    @Override
    public Translation2d image(Translation2d x) {
        if (blue())
            return x;
        return new Translation2d(m_length - x.getX(), x.getY());
    }

    /** Inverts x and theta */
    @Override
    public WindingPose2d image(WindingPose2d x) {
        if (blue())
            return x;
        return new WindingPose2d(m_length - x.getX(), x.getY(), Math.PI - x.getRotation());
    }

    @Override
    public Pose2d image(Pose2d x) {
        if (blue())
            return x;
        return new Pose2d(m_length - x.getX(), x.getY(), Rotation2d.PI.minus(x.getRotation()));
    }

    /** Inverts x and theta */
    @Override
    public DirectionSE2 image(DirectionSE2 x) {
        if (blue())
            return x;
        return new DirectionSE2(-1.0 * x.x, x.y, -1.0 * x.theta);
    }
}
