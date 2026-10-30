package org.team100.lib.localization.fused;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.testing.TestUtil;
import org.team100.lib.testing.Timeless;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.NoisyStateSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FusedEstimatorTest implements Timeless {

    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest();

    private Rotation2d yaw;
    private SwerveModulePositions positions;
    private List<VisionMeasurement> vision;

    @Test
    void testDump() throws IOException {
        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();

        NoisyPose2d measurement = new NoisyPose2d(new Pose2d(), IsotropicNoiseSE2.fromStdDev(1, 1));

        OdometryEstimator odometryEstimate = new OdometryEstimator(
                log,
                kinodynamics,
                0.04,
                0.005,
                false,
                false);
        NudgingVisionEstimator visionEstimate = new NudgingVisionEstimator(log);
        FusedEstimator estimator = new FusedEstimator(
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                odometryEstimate,
                visionEstimate);

        estimator.update(0.00);
        vision = List.of(new VisionMeasurement(0.01, measurement));
        estimator.update(0.02);
        estimator.dump();
    }

    @Test
    void test0() throws IOException {
        // straight line motion computed by odometry and
        // also imputed via pose history
        double dt = TimedRobot100.LOOP_PERIOD_S;
        LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        OdometryEstimator odometryEstimate = new OdometryEstimator(
                logger,
                swerveKinodynamics,
                0,
                1e-5,
                false,
                false);
        NudgingVisionEstimator visionEstimate = new NudgingVisionEstimator(logger);
        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();
        FusedEstimator estimate = new FusedEstimator(
                Takt.get(),
                () -> yaw,
                () -> positions,
                () -> vision,
                odometryEstimate,
                visionEstimate);

        estimate.reset(
                0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.high()));
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(),
                        new VelocitySE2(0, 0, 0)),
                estimate.getState(Takt.get()).state());

        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)));
        estimate.update(0.02);
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(0.02, 0, Rotation2d.kZero),
                        new VelocitySE2(1.0, 0, 0)),
                estimate.getState(0.02).state());

        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.04, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.04, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.04, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.04, Optional.of(Rotation2d.kZero)));
        estimate.update(0.04);
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(0.04, 0, Rotation2d.kZero),
                        new VelocitySE2(1.0, 0, 0)),
                estimate.getState(0.04).state());

        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.06, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.06, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.06, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.06, Optional.of(Rotation2d.kZero)));
        estimate.update(0.06);
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(0.06, 0, Rotation2d.kZero),
                        new VelocitySE2(1.0, 0, 0)),
                estimate.getState(0.06).state());

        Map<Double, NoisyStateSE2> all = estimate.all();
        assertEquals(4, all.size());
        List<NoisyStateSE2> states = new ArrayList<>(all.values());

        // derive velocities from the poses
        TestUtil.verify(
                new VelocitySE2(1, 0, 0),
                VelocitySE2.velocity(
                        states.get(0).pose(),
                        states.get(1).pose(), dt));
        // next states do move
        TestUtil.verify(
                new VelocitySE2(1, 0, 0),
                VelocitySE2.velocity(
                        states.get(1).pose(),
                        states.get(2).pose(), dt));
        TestUtil.verify(
                new VelocitySE2(1, 0, 0),
                VelocitySE2.velocity(
                        states.get(2).pose(),
                        states.get(3).pose(), dt));
    }
}
