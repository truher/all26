package org.team100.lib.localization.gtsam;

import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.util.TimeInterpolatableBuffer100;

/**
 * A version of the history idea that includes some of the GTSAM constraints:
 * discrete, nonqueryable/noninterpolatable estimate store.
 */
public class SeparateSwerveHistory {
    private final double m_bufferDuration;
    private final TimeInterpolatableBuffer100<OdoGyroMeasurement> m_measurements;

    public SeparateSwerveHistory(
            double bufferDuration,
            double initialTimestamp,
            OdoGyroMeasurement initialMeasurement) {
        m_bufferDuration = bufferDuration;
        m_measurements = new TimeInterpolatableBuffer100<>(
                bufferDuration, initialTimestamp, initialMeasurement);
    }
}
