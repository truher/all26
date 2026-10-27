package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.StateR1;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveStateInterpolatorTest {

    @Test
    void testInterp0() {
        // initially at rest, finally in motion.
        // what does the interpolator do?
        StateSE2 s0 = new StateSE2();
        IsotropicNoiseSE2 n0 = IsotropicNoiseSE2.fromStdDev(1, 1);
        SwerveModulePositions p0 = new SwerveModulePositions(
                new SwerveModulePosition100(0, Optional.empty()),
                new SwerveModulePosition100(0, Optional.empty()),
                new SwerveModulePosition100(0, Optional.empty()),
                new SwerveModulePosition100(0, Optional.empty()));
        Rotation2d gyroYaw0 = new Rotation2d();
        VariableR1 gyroBias = VariableR1.fromVariance(0, 0.001);
        LocalizerEstimate estimate0 = new LocalizerEstimate(s0, n0, gyroBias);
        SwerveState r0 = new SwerveState(estimate0, new OdoGyroMeasurement(gyroYaw0, p0));

        StateSE2 s1 = new StateSE2(new StateR1(), new StateR1(), new StateR1(1, 1));
        IsotropicNoiseSE2 n1 = IsotropicNoiseSE2.fromStdDev(1, 1);

        SwerveModulePositions p1 = new SwerveModulePositions(
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(-3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(-Math.PI / 4))));
        Rotation2d gyroYaw1 = new Rotation2d(1);

        // current speed is 1 rad/s
        VariableR1 gyroBias2 = VariableR1.fromVariance(0, 1);
        LocalizerEstimate estimate1 = new LocalizerEstimate(s1, n1, gyroBias2);
        SwerveState r1 = new SwerveState(estimate1, new OdoGyroMeasurement(gyroYaw1, p1));

        {
            // t=0 should return r0. note "t" is not time
            SwerveState lerp = r0.interpolate(r1, 0.0);
            LocalizerEstimate estimate = lerp.estimate();
            assertEquals(0, estimate.state().theta().x(), 0.001);
            assertEquals(0, estimate.state().theta().v(), 0.001);
            assertEquals(0, lerp.measurement().gyro().getRadians(), 0.001);

        }
        {
            // t=1 should return r1. note this is a special case.
            SwerveState lerp = r0.interpolate(r1, 1.0);
            LocalizerEstimate estimate = lerp.estimate();
            assertEquals(1, estimate.state().theta().x(), 0.001);
            assertEquals(1, estimate.state().theta().v(), 0.001);
            assertEquals(1, lerp.measurement().gyro().getRadians(), 0.001);
        }
        {
            // t=0.5 is just halfway there.
            SwerveState lerp = r0.interpolate(r1, 0.5);
            LocalizerEstimate estimate = lerp.estimate();
            assertEquals(0.5, estimate.state().theta().x(), 0.001);
            assertEquals(0.5, estimate.state().theta().v(), 0.001);
            assertEquals(0.5, lerp.measurement().gyro().getRadians(), 0.001);
        }
    }

    @Test
    void testInterp1() {
        StateSE2 s0 = new StateSE2();
        IsotropicNoiseSE2 n0 = IsotropicNoiseSE2.fromStdDev(1, 1);
        // initally driving straight
        SwerveModulePositions p0 = new SwerveModulePositions(
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())),
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())),
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())),
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())));
        Rotation2d gyroYaw0 = new Rotation2d();
        VariableR1 gyroBias = VariableR1.fromVariance(0, 1);
        LocalizerEstimate estimate0 = new LocalizerEstimate(s0, n0, gyroBias);
        SwerveState r0 = new SwerveState(estimate0, new OdoGyroMeasurement(gyroYaw0, p0));

        StateSE2 s1 = new StateSE2(new StateR1(), new StateR1(), new StateR1(1, 1));
        IsotropicNoiseSE2 n1 = IsotropicNoiseSE2.fromStdDev(1, 1);
        SwerveModulePositions p1 = new SwerveModulePositions(
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(-3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(-Math.PI / 4))));
        Rotation2d gyroYaw1 = new Rotation2d(1);

        // current speed is 1 rad/s
        VariableR1 gyroBias2 = VariableR1.fromVariance(0, 1);
        LocalizerEstimate estimate1 = new LocalizerEstimate(s1, n1, gyroBias2);
        SwerveState r1 = new SwerveState(estimate1, new OdoGyroMeasurement(gyroYaw1, p1));

        {
            // t=0 should return r0. note "t" is not time
            SwerveState lerp = r0.interpolate(r1, 0.0);
            LocalizerEstimate estimate = lerp.estimate();
            assertEquals(0, estimate.state().theta().x(), 0.001);
            assertEquals(0, estimate.state().theta().v(), 0.001);
            assertEquals(0, lerp.measurement().gyro().getRadians(), 0.001);
        }
        {
            // t=1 should return r1. note this is a special case.
            SwerveState lerp = r0.interpolate(r1, 1.0);
            LocalizerEstimate estimate = lerp.estimate();
            assertEquals(1, estimate.state().theta().x(), 0.001);
            assertEquals(1, estimate.state().theta().v(), 0.001);
            assertEquals(1, lerp.measurement().gyro().getRadians(), 0.001);
        }
        {
            // t=0.5
            SwerveState lerp = r0.interpolate(r1, 0.5);
            // wheels are assumed to turn immediately to the r1 angles,
            // so this case is the same as the case above.
            LocalizerEstimate estimate = lerp.estimate();
            assertEquals(0.5, estimate.state().theta().x(), 0.001);
            assertEquals(0.5, estimate.state().theta().v(), 0.001);
            assertEquals(0.5, lerp.measurement().gyro().getRadians(), 0.001);
        }
    }

}
