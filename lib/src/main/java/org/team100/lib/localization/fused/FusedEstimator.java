package org.team100.lib.localization.fused;

import java.util.Map;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.localization.HistoryUpdater;
import org.team100.lib.localization.ImmutableSwerveHistory;
import org.team100.lib.localization.LocalizerEstimate;
import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.localization.StateEstimator;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.localization.map.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.SwerveLocal;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Provides state estimates after updating vision and odometry.
 * 
 * The underlying updaters use "Fusors" and replay.
 * 
 * The result is stored as an immutable copy, which is a step
 * towards the GTSAM way, where the computation is elsewhere.
 */
public class FusedEstimator implements StateEstimator {
    private static final boolean DEBUG = false;
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;

    private final Gyro m_gyro;
    private final SwerveLocal m_swerveLocal;
    private final SwerveHistory m_history;
    private final AprilTagCornerReader m_tagReader;
    private final HistoryUpdater m_updater;
    private final ObjectCache<ImmutableSwerveHistory> m_immutable;

    public FusedEstimator(LoggerFactory driveLog,
            LoggerFactory fieldLogger,
            SwerveKinodynamics swerveKinodynamics,
            boolean noisy,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Gyro gyro,
            SwerveLocal swerveLocal) {
        m_gyro = gyro;
        m_swerveLocal = swerveLocal;
        m_history = new SwerveHistory(
                0.2,
                Takt.get(),
                new OdoGyroMeasurement(m_gyro.getYawNWU(), swerveLocal.positions()),
                new LocalizerEstimate(new StateSE2(Pose2d.kZero, VelocitySE2.ZERO), IsotropicNoiseSE2.high(),
                        VariableR1.fromVariance(0, 1)));
        OdometryEstimator odometryEstimate = new OdometryEstimator(
                driveLog,
                swerveKinodynamics,
                gyro.white_noise(),
                gyro.bias_noise(),
                noisy,
                false);
        OdometryReplayer or = new OdometryReplayer(m_history, odometryEstimate);
        NudgingVisionEstimator visionEstimate = new NudgingVisionEstimator(driveLog);
        m_tagReader = new AprilTagCornerReader(
                driveLog, layout, DriverStation::getAlliance);
        m_updater = new HistoryUpdater(
                m_gyro::getYawNWU,
                m_swerveLocal::positions,
                m_tagReader::read,
                m_history,
                odometryEstimate,
                or,
                visionEstimate);
        m_immutable = Cache.of(this::makeImmutable);
    }

    /** Update the history and then make a copy */
    ImmutableSwerveHistory makeImmutable() {
        // mutates history
        m_updater.update(Takt.get());
        return m_history.immutableCopy();
    }

    @Override
    public Map<Double, StateSE2> all() {
        return m_immutable.get().all();
    }

    /**
     * Estimate at the given timestamp, after applying any pending updates from
     * vision or odometry.
     * 
     * The estimate is used for many things downstream; noise there is bad.
     * The estimator itself should have enough controls to make the estimate
     * arbitrarily smooth.
     */
    @Override
    public StateSE2 getState(double timestampS) {
        // run our dependencies if they haven't already
        ImmutableSwerveHistory h = m_immutable.get();
        final StateSE2 state;
        if (Experiments.INSTANCE.enabled(Experiment.ImputeVelocity)) {
            // Use consecutive poses
            StateSE2 state0 = h.get(timestampS - DT);
            StateSE2 state1 = h.get(timestampS);
            VelocitySE2 v = VelocitySE2.velocity(
                    state0.pose(),
                    state1.pose(), DT);
            state = new StateSE2(state1.pose(), v);
        } else {
            // Use the history value
            state = h.get(timestampS);
        }
        if (DEBUG) {
            System.out.printf("FreshSwerveEstimate.update() estimated pose: %s\n", state);
        }
        return state;
    }

    /**
     * Empty the pose history, reset the servos, add the given pose, and flush the
     * cache.
     */
    @Override
    public void reset(Pose2d pose, IsotropicNoiseSE2 noise) {
        m_history.reset(
                m_swerveLocal.positions(),
                pose,
                noise,
                Takt.get(),
                m_gyro.getYawNWU(),
                VariableR1.fromVariance(0, 1));
        m_immutable.reset();
    }

    /**
     * Tags outside this radius are ignored.
     */
    @Override
    public void setHeedRadiusM(double heedRadiusM) {
        m_tagReader.setHeedRadiusM(heedRadiusM);
    }

    @Override
    public void close() {
    }

}
