package org.team100.lib.localization;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.localization.fused.NudgingVisionEstimator;
import org.team100.lib.localization.fused.OdometryEstimator;
import org.team100.lib.localization.fused.OdometryReplayer;
import org.team100.lib.localization.fused.VisionMeasurement;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VariableR1;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class HistoryUpdaterTest {
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest();

    private Rotation2d yaw;
    private SwerveModulePositions positions;
    private List<VisionMeasurement> vision;

    @Test
    void test0() {
        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        SwerveHistory history = new SwerveHistory(
                0.2,
                0,
                new OdoGyroMeasurement(Rotation2d.kZero, positions),
                new LocalizerEstimate(new StateSE2(Pose2d.kZero, VelocitySE2.ZERO), IsotropicNoiseSE2.high(),
                        VariableR1.fromVariance(0, 1)));
        OdometryEstimator odometryEstimator = new OdometryEstimator(
                log, kinodynamics, 4e-4, 1e-5, false, true);
        OdometryReplayer replayer = new OdometryReplayer(history, odometryEstimator);
        NudgingVisionEstimator visionEstimator = new NudgingVisionEstimator(log);
        NoisyPose2d measurement = new NoisyPose2d(new Pose2d(), IsotropicNoiseSE2.fromStdDev(1, 1));

        vision = List.of();
        HistoryUpdater updater = new HistoryUpdater(
                () -> yaw,
                () -> positions,
                () -> vision,
                history, odometryEstimator, replayer, visionEstimator);
        updater.update(0.00);
        vision = List.of(new VisionMeasurement(0.01, measurement));
        updater.update(0.02);
        history.dump();
    }

}
