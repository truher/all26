package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class SwerveHistoryTest {
    private static final double DELTA = 0.001;

    @Test
    void testReset() {
        SwerveHistory history = new SwerveHistory(
                0.2,
                0,
                new OdoGyroMeasurement(Rotation2d.kZero, SwerveModulePositions.ZERO),
                new LocalizerEstimate(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO), IsotropicNoiseSE2.high(),
                        VariableR1.fromVariance(0, 1)));
        history.reset(
                SwerveModulePositions.ZERO,
                new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                IsotropicNoiseSE2.high(),
                0, Rotation2d.kZero,
                VariableR1.fromVariance(0, 1));
        Pose2d p = history.get(0).pose();
        assertEquals(0, p.getX(), DELTA);
        assertEquals(0, p.getY(), DELTA);
        assertEquals(0, p.getRotation().getRadians(), DELTA);

        // force the pose to rotate 90
        history.reset(
                SwerveModulePositions.ZERO,
                new StateSE2(new Pose2d(0, 0, Rotation2d.kCCW_90deg), VelocitySE2.ZERO),
                IsotropicNoiseSE2.high(),
                0, Rotation2d.kZero,
                VariableR1.fromVariance(0, 1));
        p = history.get(0).pose();
        assertEquals(0, p.getX(), DELTA);
        assertEquals(0, p.getY(), DELTA);
        // and we get that back
        assertEquals(Math.PI / 2, p.getRotation().getRadians(), DELTA);
    }
}
