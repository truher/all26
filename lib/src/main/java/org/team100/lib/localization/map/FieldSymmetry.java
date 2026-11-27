package org.team100.lib.localization.map;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.team100.lib.geometry.se2_universal_cover.WindingWaypointSE2;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Transform positions, poses, waypoints, etc, according to the active alliance.
 * 
 * This transform enables you to design paths using the "blue" coordinates, and
 * then transform them to be used when playing the "red" side.
 * 
 * Some years, the field symmetry is reflective, other years it is rotational.
 * 
 * Do not call these methods at startup time, because the alliance
 * is not known until the FMS and DS connect. Use them in functions,
 * e.g. the trajectory function in DriveWithTrajectoryFunction.
 */
public interface FieldSymmetry {

    Translation2d image(Translation2d x);

    default Translation2d[] image(Translation2d[] x) {
        return Arrays.stream(x).map(
                p -> image(p))
                .toArray(Translation2d[]::new);
    }

    WindingPose2d image(WindingPose2d x);

    Pose2d image(Pose2d x);

    DirectionSE2 image(DirectionSE2 x);

    default WindingWaypointSE2 image(WindingWaypointSE2 x) {
        return new WindingWaypointSE2(image(x.pose()), image(x.course()), x.scale());
    }

    /**
     * IMPORTANT! Does not map the starting pose! IMPORTANT!
     * 
     * The expecation is that you'll use this for lists of waypoints that start with
     * the "current pose", and so you don't want to change it. If you want to map
     * the entire list, use `image()` below.
     */
    default List<WindingWaypointSE2> imageExclusive(List<WindingWaypointSE2> x) {
        if (x.size() < 2)
            return x;
        // Adjust the rotation so it is "near" the rest of the path.
        WindingWaypointSE2 x0 = x.get(0);
        // Leave the pose alone, adjust the course.
        WindingWaypointSE2 x0p = new WindingWaypointSE2(x0.pose(), image(x0.course()), x0.scale());
        x0p = x0p.near(image(x.get(1)));
        return Stream.concat(
                Stream.of(x0p),
                x.stream().skip(1).map(p -> image(p))).toList();
    }

    /** Maps all waypoints in the list. */
    default List<WindingWaypointSE2> image(List<WindingWaypointSE2> x) {
        return x.stream().map(p -> image(p)).toList();
    }

    public static FieldSymmetry get(
            double length,
            double width,
            Supplier<Optional<Alliance>> alliance) {
        // this year it's rotational
        return new FieldSymmetryRotational(length, width, alliance);
    }

}
