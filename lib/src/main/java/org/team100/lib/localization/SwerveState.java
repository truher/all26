package org.team100.lib.localization;

import java.util.Objects;

import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * Estimates for position and velocity in SE2, with noise, and gyro bias.
 * 
 * Measurements of gyro and wheels, and, optionally,
 */
public class SwerveState implements Interpolatable<SwerveState> {

    private final LocalizerEstimate m_estimate;

    private final OdoGyroMeasurement m_measurement;

    public SwerveState(
            LocalizerEstimate estimate,
            OdoGyroMeasurement measurement) {
        m_estimate = estimate;
        m_measurement = measurement;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof SwerveState)) {
            return false;
        }
        SwerveState rec = (SwerveState) obj;
        return Objects.equals(m_measurement, rec.m_measurement)
                && Objects.equals(m_estimate, rec.m_estimate);
    }

    public LocalizerEstimate estimate() {
        return m_estimate;
    }

    public OdoGyroMeasurement measurement() {
        return m_measurement;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_measurement, m_estimate);
    }

    @Override
    public String toString() {
        return "SwerveState [m_estimate=" + m_estimate
                + ", m_measurement=" + m_measurement + "]";
    }

    /**
     * This used to interpolate wheel positions to find a "twist" to apply
     * to the start pose, but we don't do it that way as of 10/4/26.
     * 
     * Now we just interpolate each field separately.
     */
    @Override
    public SwerveState interpolate(SwerveState endValue, double t) {
        // Check bounds on t
        if (t <= 0) {
            return this;
        }
        if (t >= 1) {
            return endValue;
        }

        OdoGyroMeasurement odoLerp = measurement().interpolate(endValue.measurement(), t);
        LocalizerEstimate estimateLerp = estimate().interpolate(endValue.estimate(), t);
        return new SwerveState(estimateLerp, odoLerp);
    }

}