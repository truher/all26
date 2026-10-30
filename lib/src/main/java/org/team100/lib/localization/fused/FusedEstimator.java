package org.team100.lib.localization.fused;

import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Supplier;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.localization.ImmutableSwerveHistory;
import org.team100.lib.localization.LocalizerEstimate;
import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.localization.StateEstimator;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.NoisyStateSE2;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Provides state estimates after updating vision and odometry, using "Fusors"
 * and replay.
 * 
 * The result is stored as an immutable copy, which is a step towards the GTSAM
 * way, where the computation is elsewhere.
 * 
 * This estimate is not self-updating (to make it easier to test).  If you want
 * that, wrap it with UpdatedEstimate.
 */
public class FusedEstimator implements StateEstimator {
    private static final boolean DEBUG = false;
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;

    private final Supplier<Rotation2d> m_gyro;
    private final Supplier<SwerveModulePositions> m_positions;
    private final Supplier<List<VisionMeasurement>> m_vision;
    private final OdometryEstimator m_odometryEstimator;
    private final NudgingVisionEstimator m_visionEstimator;
    private final SwerveHistory m_history;
    private final OdometryReplayer m_replayer;

    public FusedEstimator(
            double initialTimestampS,
            Supplier<Rotation2d> gyro,
            Supplier<SwerveModulePositions> positions,
            Supplier<List<VisionMeasurement>> vision,
            OdometryEstimator odometryEstimate,
            NudgingVisionEstimator visionEstimate) {
        m_gyro = gyro;
        m_positions = positions;
        m_vision = vision;
        m_odometryEstimator = odometryEstimate;
        m_visionEstimator = visionEstimate;
        m_history = new SwerveHistory(
                0.2,
                initialTimestampS,
                new OdoGyroMeasurement(m_gyro.get(), positions.get()),
                new LocalizerEstimate(new StateSE2(Pose2d.kZero, VelocitySE2.ZERO), IsotropicNoiseSE2.high(),
                        VariableR1.fromVariance(0, 1)));
        m_replayer = new OdometryReplayer(m_history, odometryEstimate);
    }

    public ImmutableSwerveHistory immutableCopy() {
        return m_history.immutableCopy();
    }

    /**
     * Read pending updates from odo/gyro and vision, and apply them to the history,
     * with replay. TODO: reduce visibility
     */
    public ImmutableSwerveHistory update(double timestamp) {
        odoGyro(timestamp);
        vision();
        return m_history.immutableCopy();
    }

    @Override
    public Map<Double, NoisyStateSE2> all() {
        return m_history.immutableCopy().all();
    }

    @Override
    public NoisyStateSE2 getState(double timestampS) {
        // run our dependencies if they haven't already
        ImmutableSwerveHistory h = m_history.immutableCopy();
        final NoisyStateSE2 state;
        if (Experiments.INSTANCE.enabled(Experiment.ImputeVelocity)) {
            // Use consecutive poses
            NoisyStateSE2 state0 = h.get(timestampS - DT);
            NoisyStateSE2 state1 = h.get(timestampS);
            VelocitySE2 v = VelocitySE2.velocity(
                    state0.pose(),
                    state1.pose(), DT);
            state = new NoisyStateSE2(
                    new StateSE2(state1.pose(), v), state1.noise());
        } else {
            // Use the history value
            state = h.get(timestampS);
        }
        if (DEBUG) {
            System.out.printf("FreshSwerveEstimate.update() estimated pose: %s\n", state);
        }
        return state;
    }

    @Override
    public void reset(double timestampS, NoisyStateSE2 state) {
        m_history.reset(
                new OdoGyroMeasurement(m_gyro.get(), m_positions.get()),
                new LocalizerEstimate(
                        state.state(),
                        state.noise(),
                        VariableR1.fromVariance(0, 1)),

                timestampS);
    }

    @Override
    public void close() {
    }

    /** For testing. */
    public void dump() {
        m_history.dump();
    }

    /**
     * Add the one-and-only odo/gyro measurement for this timestamp.
     */
    private void odoGyro(double timestamp) {
        Entry<Double, LocalizerEstimate> estimateEntry = m_history
                .estimateLowerEntry(timestamp);
        if (estimateEntry == null)
            return;
        LocalizerEstimate previousEstimate = estimateEntry.getValue();

        Entry<Double, OdoGyroMeasurement> measurementEntry = m_history
                .measurementLowerEntry(timestamp);
        if (measurementEntry == null)
            return;
        Double previousTimestamp = measurementEntry.getKey();
        OdoGyroMeasurement previousMeasurement = measurementEntry.getValue();

        OdoGyroMeasurement newMeasurement = new OdoGyroMeasurement(m_gyro.get(), m_positions.get());
        LocalizerEstimate newEstimate = m_odometryEstimator.estimate(
                previousTimestamp,
                previousEstimate,
                previousMeasurement,
                timestamp,
                newMeasurement);
        m_history.put(timestamp, newEstimate, newMeasurement);
    }

    /**
     * Apply pending vision updates.
     */
    private void vision() {
        for (VisionMeasurement v : m_vision.get()) {
            double visionTimestamp = v.timestamp();
            if (m_history.tooOld(visionTimestamp))
                continue;
            NoisyPose2d visionMeasurement = v.noisyMeasurement();
            OdoGyroMeasurement sampleMeasurement = m_history.getOdoGyroMeasurement(visionTimestamp);
            LocalizerEstimate sampleEstimate = m_history.getLocalizerEstimate(visionTimestamp);
            LocalizerEstimate newEstimate = m_visionEstimator.estimate(
                    sampleEstimate, visionMeasurement);
            m_history.put(visionTimestamp, newEstimate, sampleMeasurement);
            m_replayer.replay(visionTimestamp);
        }
    }
}
