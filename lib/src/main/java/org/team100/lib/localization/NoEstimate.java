package org.team100.lib.localization;

import java.util.Map;
import java.util.TreeMap;

import org.team100.lib.state.StateSE2;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyStateSE2;

/** Placeholder state estimator */
public class NoEstimate implements StateEstimator {

    @Override
    public NoisyStateSE2 getState(double timestampS) {
        return new NoisyStateSE2(new StateSE2(), IsotropicNoiseSE2.high());
    }

    @Override
    public void reset(double timestampS, NoisyStateSE2 state) {
    }

    @Override
    public Map<Double, NoisyStateSE2> all() {
        return new TreeMap<>();
    }

    @Override
    public void close() {
    }

}
