package org.team100.lib.localization;

import java.util.Iterator;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

import org.team100.lib.localization.fused.NudgingVisionEstimator;
import org.team100.lib.localization.fused.OdometryEstimator;
import org.team100.lib.localization.fused.OdometryReplayer;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.NoisyPose2d;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Centralized batched pose-estimation updates.
 */
public class HistoryGardener {
    private final NavigableMap<Double, NoisyPose2d> m_pendingVision;
    private final NavigableMap<Double, OdoGyroMeasurement> m_pendingOdo;

    private final SwerveHistory m_history;
    private final OdometryEstimator m_OdometryEstimator;
    private final OdometryReplayer m_replayer;
    private final NudgingVisionEstimator m_visionUpdater;

    public HistoryGardener(
            SwerveHistory history,
            OdometryEstimator OdometryEstimator,
            OdometryReplayer replayer,
            NudgingVisionEstimator visionEstimate) {
        m_pendingVision = new ConcurrentSkipListMap<>();
        m_pendingOdo = new ConcurrentSkipListMap<>();
        m_history = history;
        m_OdometryEstimator = OdometryEstimator;
        m_replayer = replayer;
        m_visionUpdater = visionEstimate;
    }

    /** Add pending odometry measurement. */
    public void putOdometry(
            double timeSec,
            Rotation2d gyroYaw,
            SwerveModulePositions positions) {
        m_pendingOdo.put(timeSec, new OdoGyroMeasurement(gyroYaw, positions));
    }

    /** Add pending vision measurement. */
    public void putVision(
            double timeSec,
            NoisyPose2d measurement) {
        m_pendingVision.put(timeSec, measurement);
    }

    /** Handle pending updates in time order. */
    public void sweep() {
        Iterator<Entry<Double, OdoGyroMeasurement>> odoIter = m_pendingOdo.entrySet().iterator();
        Iterator<Entry<Double, NoisyPose2d>> visionIter = m_pendingVision.entrySet().iterator();

        Entry<Double, OdoGyroMeasurement> odo = odoIter.hasNext() ? odoIter.next() : null;
        Entry<Double, NoisyPose2d> vision = visionIter.hasNext() ? visionIter.next() : null;

        while (odo != null && vision != null) {
            double visionTimestamp = vision.getKey();
            NoisyPose2d visionMeasurement = vision.getValue();
            double odoTimestamp = odo.getKey();
            OdoGyroMeasurement odoMeasurement = odo.getValue();
            if (odoTimestamp < visionTimestamp) {
                Entry<Double, LocalizerEstimate> estimateLowerEntry = m_history
                        .localizerEstimateLowerEntry(odoTimestamp);
                Entry<Double, OdoGyroMeasurement> measurementLowerEntry = m_history.swerveStateLowerEntry(odoTimestamp);
                if (estimateLowerEntry != null && measurementLowerEntry != null) {
                    LocalizerEstimate e = m_OdometryEstimator.estimate(
                            measurementLowerEntry.getKey(),
                            estimateLowerEntry.getValue(),
                            measurementLowerEntry.getValue(),
                            odoTimestamp,
                            odoMeasurement);
                    SwerveState s = new SwerveState(e, odoMeasurement);
                    if (e != null)
                        m_history.put(odoTimestamp, s);
                }
                odo = odoIter.hasNext() ? odoIter.next() : null;
            } else {
                OdoGyroMeasurement sample = m_history.getOdoGyroMeasurement(visionTimestamp);
                LocalizerEstimate sampleEstimate = m_history.getLocalizerEstimate(visionTimestamp);
                LocalizerEstimate estimate = m_visionUpdater.estimate(
                        sampleEstimate, visionMeasurement);
                SwerveState s = new SwerveState(estimate, sample);
                if (estimate != null && !m_history.tooOld(visionTimestamp)) {
                    m_history.put(visionTimestamp, s);
                    m_replayer.replay(visionTimestamp);
                }
                vision = visionIter.hasNext() ? visionIter.next() : null;
            }
        }
        // catch the remaining; one of these will work.
        while (odo != null) {
            double odoTimestamp = odo.getKey();
            OdoGyroMeasurement odoMeasurement = odo.getValue();
            Entry<Double, LocalizerEstimate> estimateLowerEntry = m_history
                    .localizerEstimateLowerEntry(odoTimestamp);
            Entry<Double, OdoGyroMeasurement> measurementLowerEntry = m_history.swerveStateLowerEntry(odoTimestamp);
            if (estimateLowerEntry != null && measurementLowerEntry != null) {
                LocalizerEstimate e = m_OdometryEstimator.estimate(
                        measurementLowerEntry.getKey(),
                        estimateLowerEntry.getValue(),
                        measurementLowerEntry.getValue(),
                        odoTimestamp,
                        odoMeasurement);
                SwerveState s = new SwerveState(e, odoMeasurement);
                if (e != null)
                    m_history.put(odoTimestamp, s);
            }
            odo = odoIter.hasNext() ? odoIter.next() : null;
        }
        while (vision != null) {
            double visionTimestamp = vision.getKey();
            NoisyPose2d visionMeasurement = vision.getValue();
            OdoGyroMeasurement sample = m_history.getOdoGyroMeasurement(visionTimestamp);
            LocalizerEstimate sampleEstimate = m_history.getLocalizerEstimate(visionTimestamp);
            LocalizerEstimate estimate = m_visionUpdater.estimate(
                    sampleEstimate, visionMeasurement);
            SwerveState s = new SwerveState(
                    estimate,
                    sample);
            if (estimate != null && !m_history.tooOld(visionTimestamp)) {
                m_history.put(visionTimestamp, s);
                m_replayer.replay(visionTimestamp);
            }
            vision = visionIter.hasNext() ? visionIter.next() : null;
        }

        m_pendingOdo.clear();
        m_pendingVision.clear();
    }
}
