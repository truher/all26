package org.team100.lib.localization.fused;

import java.util.List;

public interface VisionMeasurementSource {
    /** Read all available measurements. */
    List<VisionMeasurement> read();

    /** Truncate the queue of measurement inputs. */
    void truncate();
}
