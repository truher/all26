package org.team100.lib.localization.gtsam;

import java.util.List;

public interface CornerSource {
    /** Read all available measurements. */

    List<Corners> read();

    /** Truncate the queue of measurement inputs. */
    void truncate();
}
