package org.team100.lib.localization.fused;

import java.util.Map;
import java.util.Map.Entry;

import org.team100.lib.localization.LocalizerEstimate;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.localization.SwerveState;

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
        for (Map.Entry<Double, SwerveState> entry : m_history.swerveStateExclusiveTailMap(sampleTime).entrySet()) {
            double timestamp = entry.getKey();
            SwerveState value = entry.getValue();
            Entry<Double, SwerveState> lowerEntry = m_history.swerveStateLowerEntry(timestamp);
            if (lowerEntry != null) {
                LocalizerEstimate e = m_updater.estimate(
                        lowerEntry.getKey(), lowerEntry.getValue(),
                        timestamp, value.measurement());
                SwerveState s = new SwerveState(e, value.measurement());
                if (e != null)
                    m_history.put(timestamp, s);
            }
        }
        if (DEBUG)
            System.out.printf("==== DONE REPLAYING FOR TIME %f\n", sampleTime);
    }

}
