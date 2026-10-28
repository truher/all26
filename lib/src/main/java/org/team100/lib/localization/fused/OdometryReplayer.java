package org.team100.lib.localization.fused;

import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

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
        Set<Entry<Double, OdoGyroMeasurement>> entriesAfterSampleTime = m_history
                .swerveStateExclusiveTailMap(sampleTime).entrySet();
        for (Map.Entry<Double, OdoGyroMeasurement> entry : entriesAfterSampleTime) {
            double timestamp = entry.getKey();

            Entry<Double, LocalizerEstimate> estimateEntry = m_history.localizerEstimateLowerEntry(timestamp);
            if (estimateEntry == null)
                continue;
            LocalizerEstimate previousEstimate = estimateEntry.getValue();

            Entry<Double, OdoGyroMeasurement> measurementEntry = m_history.odoGyroMeasurementLowerEntry(timestamp);
            if (measurementEntry == null)
                continue;
            double previousTimestamp = measurementEntry.getKey();
            OdoGyroMeasurement previousMeasurement = measurementEntry.getValue();

            OdoGyroMeasurement newMeasurement = entry.getValue();

            LocalizerEstimate newEstimate = m_updater.estimate(
                    previousTimestamp,
                    previousEstimate,
                    previousMeasurement,
                    timestamp,
                    newMeasurement);
            m_history.put(timestamp, newEstimate, newMeasurement);
        }
        if (DEBUG)
            System.out.printf("==== DONE REPLAYING FOR TIME %f\n", sampleTime);
    }

}
