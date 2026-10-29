package org.team100.lib.localization;

import java.util.Map;
import java.util.TreeMap;

import org.team100.lib.state.StateSE2;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;

/** Placeholder state estimator */
public class NoEstimate implements StateEstimator {

    @Override
    public StateSE2 getState(double timestampS) {
        return new StateSE2();
    }

    @Override
    public void reset(StateSE2 pose, IsotropicNoiseSE2 noise) {
    }

    @Override
    public void setHeedRadiusM(double heedRadiusM) {
    }

    @Override
    public Map<Double, StateSE2> all() {
        return new TreeMap<>();
    }

    @Override
    public void close() {
    }

}
