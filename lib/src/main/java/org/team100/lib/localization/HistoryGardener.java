package org.team100.lib.localization;

import java.util.Iterator;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.NoisyPose2d;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Centralized batched pose-estimation updates.
 */
public class HistoryGardener {
    record Odo(Rotation2d gyro, SwerveModulePositions odo) {
    }

    private final NavigableMap<Double, NoisyPose2d> m_pendingVision;
    private final NavigableMap<Double, Odo> m_pendingOdo;

    private final SwerveHistory m_history;
    private final OdometryUpdater m_odometryUpdater;
    private final NudgingVisionUpdater m_visionUpdater;

    public HistoryGardener(
            SwerveHistory history,
            OdometryUpdater odometryUpdater,
            NudgingVisionUpdater visionUpdater) {
        m_pendingVision = new ConcurrentSkipListMap<>();
        m_pendingOdo = new ConcurrentSkipListMap<>();
        m_history = history;
        m_odometryUpdater = odometryUpdater;
        m_visionUpdater = visionUpdater;
    }

    /** Add pending odometry measurement. */
    public void putOdometry(
            double timeSec,
            Rotation2d gyroYaw,
            SwerveModulePositions positions) {
        m_pendingOdo.put(timeSec, new Odo(gyroYaw, positions));
    }

    /** Add pending vision measurement. */
    public void putVision(
            double timeSec,
            NoisyPose2d measurement) {
        m_pendingVision.put(timeSec, measurement);
    }

    /** Handle pending updates in time order. */
    public void sweep() {
        Iterator<Entry<Double, Odo>> odoIter = m_pendingOdo.entrySet().iterator();
        Iterator<Entry<Double, NoisyPose2d>> visionIter = m_pendingVision.entrySet().iterator();

        Entry<Double, Odo> odo = odoIter.hasNext() ? odoIter.next() : null;
        Entry<Double, NoisyPose2d> vision = visionIter.hasNext() ? visionIter.next() : null;

        while (odo != null && vision != null) {
            if (odo.getKey() < vision.getKey()) {
                SwerveState s = m_odometryUpdater.estimate(odo.getKey(), odo.getValue().gyro, odo.getValue().odo);
                if (s != null)
                    m_history.put(odo.getKey(), s);
                odo = odoIter.hasNext() ? odoIter.next() : null;
            } else {
                m_visionUpdater.put(vision.getKey(), vision.getValue());
                vision = visionIter.hasNext() ? visionIter.next() : null;
            }
        }
        // catch the remaining; one of these will work.
        while (odo != null) {
            SwerveState s = m_odometryUpdater.estimate(odo.getKey(), odo.getValue().gyro, odo.getValue().odo);
            if (s != null)
                m_history.put(odo.getKey(), s);
            odo = odoIter.hasNext() ? odoIter.next() : null;
        }
        while (vision != null) {
            m_visionUpdater.put(vision.getKey(), vision.getValue());
            vision = visionIter.hasNext() ? visionIter.next() : null;
        }

        m_pendingOdo.clear();
        m_pendingVision.clear();
    }
}
