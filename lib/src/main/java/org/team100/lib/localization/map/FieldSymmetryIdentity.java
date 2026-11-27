package org.team100.lib.localization.map;

import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

public class FieldSymmetryIdentity implements FieldSymmetry {

    @Override
    public Translation2d image(Translation2d x) {
        return x;
    }

    @Override
    public Translation2d[] image(Translation2d[] x) {
        return x;
    }

    @Override
    public WindingPose2d image(WindingPose2d x) {
        return x;
    }

    @Override
    public Pose2d image(Pose2d x) {
        return x;
    }

    @Override
    public DirectionSE2 image(DirectionSE2 x) {
        return x;
    }
}
