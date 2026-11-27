package org.team100.lib.localization.map;

import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.team100.lib.geometry.se2_universal_cover.WindingWaypointSE2;
import org.team100.lib.testing.TestUtil;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Translation2d;

public class FieldSymmetryTest {
    @Test
    void test0() {
        Translation2d x = new Translation2d(1, 1);
        FieldSymmetry s = new FieldSymmetryRotational(16, 8, () -> Optional.of(Alliance.BLUE));
        TestUtil.verify(new Translation2d(1, 1), s.image(x));
    }

    @Test
    void test1() {
        Translation2d x = new Translation2d(1, 1);
        FieldSymmetry s = new FieldSymmetryRotational(16, 8, () -> Optional.of(Alliance.RED));
        TestUtil.verify(new Translation2d(15, 7), s.image(x));
    }

    @Test
    void test2() {
        Translation2d[] x = new Translation2d[] {
                new Translation2d(1, 2),
                new Translation2d(3, 4) };
        FieldSymmetry s = new FieldSymmetryRotational(16, 8, () -> Optional.of(Alliance.BLUE));
        TestUtil.verify(
                new Translation2d[] {
                        new Translation2d(1, 2),
                        new Translation2d(3, 4) },
                s.image(x));
    }

    @Test
    void test3() {
        Translation2d[] x = new Translation2d[] {
                new Translation2d(1, 2),
                new Translation2d(3, 4) };
        FieldSymmetry s = new FieldSymmetryRotational(16, 8, () -> Optional.of(Alliance.RED));
        TestUtil.verify(
                new Translation2d[] {
                        new Translation2d(15, 6),
                        new Translation2d(13, 4) },
                s.image(x));
    }

    @Test
    void test4() {
        WindingPose2d x = new WindingPose2d(1, 1, 1);
        FieldSymmetry s = new FieldSymmetryRotational(16, 8, () -> Optional.of(Alliance.RED));
        TestUtil.verify(new WindingPose2d(15, 7, 4.142), s.image(x));
    }

    @Test
    void test5() {
        DirectionSE2 x = new DirectionSE2(1, 1, 1);
        FieldSymmetry s = new FieldSymmetryRotational(16, 8, () -> Optional.of(Alliance.RED));
        TestUtil.verify(new DirectionSE2(-1, -1, 1), s.image(x));
    }

    @Test
    void test6() {
        WindingWaypointSE2 x = new WindingWaypointSE2(
                new WindingPose2d(1, 1, 1),
                new DirectionSE2(1, 1, 1), 1);
        FieldSymmetry s = new FieldSymmetryRotational(16, 8, () -> Optional.of(Alliance.RED));
        TestUtil.verify(new WindingWaypointSE2(
                new WindingPose2d(15, 7, 4.142),
                new DirectionSE2(-1, -1, 1), 1), s.image(x));
    }

    @Test
    void test7() {
        List<WindingWaypointSE2> x = List.of(
                new WindingWaypointSE2(
                        new WindingPose2d(1, 1, 1),
                        new DirectionSE2(1, 1, 1), 1),
                new WindingWaypointSE2(
                        new WindingPose2d(2, 2, 2),
                        new DirectionSE2(1, 1, 1), 1));
        FieldSymmetry s = new FieldSymmetryRotational(16, 8, () -> Optional.of(Alliance.RED));
        TestUtil.verify(
                List.of(
                        new WindingWaypointSE2(
                                new WindingPose2d(15, 7, 4.142),
                                new DirectionSE2(-1, -1, 1), 1),
                        new WindingWaypointSE2(
                                new WindingPose2d(14, 6, 5.142),
                                new DirectionSE2(-1, -1, 1), 1)),
                s.image(x));
    }

    @Test
    void test8() {
        // the first rotation stays where it is
        // the second rotation goes to ~5
        // the first rotation wants to be near to that, so it chooses ~7.
        List<WindingWaypointSE2> x = List.of(
                new WindingWaypointSE2(
                        new WindingPose2d(1, 1, 1),
                        new DirectionSE2(1, 1, 1), 1),
                new WindingWaypointSE2(
                        new WindingPose2d(2, 2, 2),
                        new DirectionSE2(1, 1, 1), 1));
        FieldSymmetry s = new FieldSymmetryRotational(16, 8, () -> Optional.of(Alliance.RED));
        TestUtil.verify(
                List.of(
                        new WindingWaypointSE2(
                                new WindingPose2d(1, 1, 7.283),
                                new DirectionSE2(-1, -1, 1), 1),
                        new WindingWaypointSE2(
                                new WindingPose2d(14, 6, 5.142),
                                new DirectionSE2(-1, -1, 1), 1)),
                s.imageExclusive(x));
    }
}
