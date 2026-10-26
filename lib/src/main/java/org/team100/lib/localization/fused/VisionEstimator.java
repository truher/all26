package org.team100.lib.localization.fused;

import org.team100.lib.localization.SwerveState;
import org.team100.lib.uncertainty.NoisyPose2d;

/** For testing. */
public interface VisionEstimator {

    /**
     * Estimate SwerveState based on the supplied measurement.
     * 
     * Caller should replay if desired.
     */
    SwerveState estimate(double timestampS, NoisyPose2d noisyMeasurement);

}