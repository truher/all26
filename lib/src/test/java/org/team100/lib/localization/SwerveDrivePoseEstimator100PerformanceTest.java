package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Map.Entry;
import java.util.Optional;

import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.localization.fused.NudgingVisionEstimator;
import org.team100.lib.localization.fused.OdometryEstimator;
import org.team100.lib.localization.fused.OdometryReplayer;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.MockGyro;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VariableR1;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class SwerveDrivePoseEstimator100PerformanceTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    private final Pose2d visionRobotPoseMeters = new Pose2d(1, 0, Rotation2d.kZero);

    static SwerveModulePositions p(double x) {
        SwerveModulePosition100 m = new SwerveModulePosition100(x, Optional.of(Rotation2d.kZero));
        return new SwerveModulePositions(m, m, m, m);
    }

    private SwerveModulePositions positions;

    /*
     * Should we optimize the pose replay operation?
     * 
     * On my machine, an i5-9400f, 2.9ghz, each update takes about 3 microseconds.
     * 
     * The RoboRIO is about 3.5x slower than my machine, so say 10 microseconds per
     * update.
     * 
     * Each camera may provide zero or one updates per roboRIO cycle, so the worst
     * case would be 50 microseconds for these pose buffer updates.
     * 
     * The total budget is 20000 microseconds, but it would be good to be way under
     * that, so say 5000 microseconds. So pose buffer updates may consume 1% of
     * the budget in the worst case.
     * 
     * The average case is about 3 frames per cycle, so 30 microseconds or 0.6%,
     * which is significant but maybe not urgent?
     * 
     * If we *did* want to optimize it, we could move the replay from the vision
     * writer to the pose reader, and save something like 20 us (0.4%) on average.
     */
    // This takes a long time. There's no need to run this all the time
    // @Test
    void test0() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);

        Gyro gyro = new MockGyro();
        SwerveHistory history = new SwerveHistory(
                0.2,
                0,
                new OdoGyroMeasurement(Rotation2d.kZero, SwerveModulePositions.ZERO),
                new LocalizerEstimate(new StateSE2(Pose2d.kZero, VelocitySE2.ZERO), IsotropicNoiseSE2.high(),
                        VariableR1.fromVariance(0, 1)));
        positions = p(0);
        OdometryEstimator ou = new OdometryEstimator(
                logger, kinodynamics, gyro.white_noise(),
                gyro.bias_noise(),
                false, true);
        history.reset(
                positions,
                new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                IsotropicNoiseSE2.high(),
                0, gyro.getYawNWU(),
                VariableR1.fromVariance(0, 1));
        OdometryReplayer or = new OdometryReplayer(history, ou);
        NudgingVisionEstimator vu = new NudgingVisionEstimator(logger);

        // fill the buffer with odometry
        double t = 0.0;
        double duration = 0.2; // SwerveDrivePoseEstimator100.BUFFER_DURATION;
        while (t < duration) {
            positions = p(t);

            Entry<Double, LocalizerEstimate> estimateLowerEntry = history.localizerEstimateLowerEntry(t);
            Entry<Double, OdoGyroMeasurement> measurementLowerEntry = history.odoGyroMeasurementLowerEntry(t);
            if (measurementLowerEntry != null && estimateLowerEntry != null) {
                OdoGyroMeasurement newMeasurement = new OdoGyroMeasurement(gyro.getYawNWU(), positions);
                LocalizerEstimate e = ou.estimate(
                        measurementLowerEntry.getKey(),
                        estimateLowerEntry.getValue(),
                        measurementLowerEntry.getValue(),
                        t,
                        newMeasurement);
                history.put(t, e, newMeasurement);
            }
            t += 0.02;
        }
        assertEquals(11, history.size());
        assertEquals(0.2, history.lastKey(), DELTA);

        // add a very old vision estimate, which triggers replay of the entire buffer.
        int iterations = 100000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            LocalizerEstimate sampleEstimate = history.getLocalizerEstimate(0.00);
            OdoGyroMeasurement sample = history.getOdoGyroMeasurement(0.00);
            NoisyPose2d noisyMeasurement = new NoisyPose2d(visionRobotPoseMeters, visionMeasurementStdDevs);
            LocalizerEstimate estimate = vu.estimate(
                    sampleEstimate, noisyMeasurement);
            history.put(0, estimate, sample);
            or.replay(0);
        }
        long finishTime = System.currentTimeMillis();
        if (DEBUG) {
            System.out.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
            System.out.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
        }
        assertEquals(11, history.size());
        assertEquals(0.2, history.lastKey(), DELTA);

        // add a recent vision estimate, which triggers replay of a few samples.
        iterations = 1000000;
        startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            LocalizerEstimate sampleEstimate = history.getLocalizerEstimate(duration - 0.1);
            OdoGyroMeasurement sample = history.getOdoGyroMeasurement(duration - 0.1);
            NoisyPose2d noisyMeasurement = new NoisyPose2d(visionRobotPoseMeters, visionMeasurementStdDevs);
            LocalizerEstimate estimate = vu.estimate(sampleEstimate, noisyMeasurement);
            history.put(duration - 0.1, estimate, sample);
            or.replay(duration - 0.1);
        }
        finishTime = System.currentTimeMillis();
        if (DEBUG) {
            System.out.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
            System.out.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
        }
        assertEquals(11, history.size());
        assertEquals(0.2, history.lastKey(), DELTA);
    }
}
