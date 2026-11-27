package org.team100.lib.localization.map;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

public class FieldSymmetryRotational implements FieldSymmetry {

    private final double m_length;
    private final double m_width;
    private final Supplier<Optional<Alliance>> m_alliance;

    public FieldSymmetryRotational(
            double length,
            double width,
            Supplier<Optional<Alliance>> alliance) {
        m_length = length;
        m_width = width;
        m_alliance = alliance;
    }

    private boolean blue() {
        return this.m_alliance.get().orElse(Alliance.BLUE) == Alliance.BLUE;
    }

    /** Inverts x and y */
    @Override
    public Translation2d image(Translation2d x) {
        if (blue())
            return x;
        return new Translation2d(m_length - x.getX(), m_width - x.getY());
    }

    /**
     * Inverts x and y, adds pi to theta.
     * Note this will often put theta beyond pi.
     */
    @Override
    public WindingPose2d image(WindingPose2d x) {
        if (blue())
            return x;
        return new WindingPose2d(m_length - x.getX(), m_width - x.getY(), Math.PI + x.getRotation());
    }

    @Override
    public Pose2d image(Pose2d x) {
        if (blue())
            return x;
        return new Pose2d(m_length - x.getX(), m_width - x.getY(), GeometryUtil.flip(x.getRotation()));
    }

    /** Inverts x and y */
    @Override
    public DirectionSE2 image(DirectionSE2 x) {
        if (blue())
            return x;
        return new DirectionSE2(-1.0 * x.x, -1.0 * x.y, x.theta);
    }
}
