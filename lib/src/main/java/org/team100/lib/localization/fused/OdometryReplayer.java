package org.team100.lib.localization.fused;

import java.util.Map;
import java.util.Map.Entry;

import org.team100.lib.localization.LocalizerEstimate;
import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.localization.SwerveHistory;

/**
 * Replay odometry after the sample time, modifying the history.
 */
public class OdometryReplayer {
    private static final boolean DEBUG = false;
    private final SwerveHistory m_history;
    private final OdometryEstimator m_updater;

    public OdometryReplayer(SwerveHistory history, OdometryEstimator updater) {
        m_history = history;
        m_updater = updater;
    }

    public void replay(double sampleTime) {
        if (DEBUG)
            System.out.printf("==== REPLAY FOR TIME %f\n", sampleTime);
        // Note the exclusive tailmap: we don't see the entry at timestamp.
        for (Map.Entry<Double, OdoGyroMeasurement> entry : m_history.swerveStateExclusiveTailMap(sampleTime)
                .entrySet()) {
            double timestamp = entry.getKey();
            OdoGyroMeasurement value = entry.getValue();
            Entry<Double, LocalizerEstimate> estimateLowerEntry = m_history.localizerEstimateLowerEntry(timestamp);
            Entry<Double, OdoGyroMeasurement> measurementLowerEntry = m_history.odoGyroMeasurementLowerEntry(timestamp);
            if (estimateLowerEntry != null && measurementLowerEntry != null) {
                LocalizerEstimate e = m_updater.estimate(
                        measurementLowerEntry.getKey(),
                        estimateLowerEntry.getValue(),
                        measurementLowerEntry.getValue(),
                        timestamp,
                        value);
                if (e != null)
                    m_history.put(timestamp, e, value);
            }
        }
        if (DEBUG)
            System.out.printf("==== DONE REPLAYING FOR TIME %f\n", sampleTime);
    }

}
