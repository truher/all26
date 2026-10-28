package org.team100.lib.localization;

import java.util.List;
import java.util.Map.Entry;
import java.util.function.Supplier;

import org.team100.lib.localization.fused.NudgingVisionEstimator;
import org.team100.lib.localization.fused.OdometryEstimator;
import org.team100.lib.localization.fused.OdometryReplayer;
import org.team100.lib.localization.fused.VisionMeasurement;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.NoisyPose2d;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Update history with odo/gyro and vision input.
 * 
 * This used to take care to apply vision updates in time-order, in order to
 * minimize the amount of discarded replay output, but I think it's not
 * worth the complexity; the replaying is brief regardless.
 */
public class HistoryUpdater {
    private final Supplier<Rotation2d> m_gyro;
    private final Supplier<SwerveModulePositions> m_positions;
    private final Supplier<List<VisionMeasurement>> m_vision;
    private final SwerveHistory m_history;
    private final OdometryEstimator m_OdometryEstimator;
    private final OdometryReplayer m_replayer;
    private final NudgingVisionEstimator m_visionUpdater;

    public HistoryUpdater(
            Supplier<Rotation2d> gyro,
            Supplier<SwerveModulePositions> positions,
            Supplier<List<VisionMeasurement>> vision,
            SwerveHistory history,
            OdometryEstimator odometryEstimate,
            OdometryReplayer replayer,
            NudgingVisionEstimator visionEstimate) {
        m_gyro = gyro;
        m_positions = positions;
        m_vision = vision;
        m_history = history;
        m_OdometryEstimator = odometryEstimate;
        m_replayer = replayer;
        m_visionUpdater = visionEstimate;
    }

    /**
     * Add current measurements for odometry and gyro using the given timestamp.
     * Add pending vision measurements and replay.
     */
    public void update(double timestamp) {
        odoGyro(timestamp);
        vision();
    }

    /**
     * Add the one-and-only odo/gyro measurement for this timestamp.
     */
    private void odoGyro(double timestamp) {
        Entry<Double, LocalizerEstimate> estimateEntry = m_history
                .localizerEstimateLowerEntry(timestamp);
        if (estimateEntry == null)
            return;
        LocalizerEstimate previousEstimate = estimateEntry.getValue();

        Entry<Double, OdoGyroMeasurement> measurementEntry = m_history
                .odoGyroMeasurementLowerEntry(timestamp);
        if (measurementEntry == null)
            return;
        Double previousTimestamp = measurementEntry.getKey();
        OdoGyroMeasurement previousMeasurement = measurementEntry.getValue();

        OdoGyroMeasurement newMeasurement = new OdoGyroMeasurement(m_gyro.get(), m_positions.get());
        LocalizerEstimate newEstimate = m_OdometryEstimator.estimate(
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
            LocalizerEstimate newEstimate = m_visionUpdater.estimate(
                    sampleEstimate, visionMeasurement);
            m_history.put(visionTimestamp, newEstimate, sampleMeasurement);
            m_replayer.replay(visionTimestamp);
        }
    }

}
