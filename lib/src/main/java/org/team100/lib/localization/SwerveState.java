package org.team100.lib.localization;

import java.util.Objects;

import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.NoisyPose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * Estimates for position and velocity in SE2, with noise, and gyro bias.
 * 
 * Measurements of gyro and wheels, and, optionally,
 */
public class SwerveState implements Interpolatable<SwerveState> {

    private final LocalizerEstimate m_estimate;

    //
    // MEASUREMENTS
    //
    /**
     * Verbatim measurement of wheel position and angle. This may be a ground-truth
     * measurement or it may be an interpolation created to match a vision
     * timestamp.
     */
    private final SwerveModulePositions m_positions;
    /** Verbatim measurement of yaw from the gyro, uncorrected. */
    private final Rotation2d m_gyroYaw;
    /** Verbatim measurement from camera, null for odometry-derived entries. */
    private final NoisyPose2d m_visionMeasurement;

    public SwerveState(
            LocalizerEstimate estimate,
            SwerveModulePositions positions,
            Rotation2d gyroYaw,
            NoisyPose2d visionMeasurement) {
        m_estimate = estimate;
        // MEASUREMENTS
        m_positions = positions;
        m_gyroYaw = gyroYaw;
        m_visionMeasurement = visionMeasurement;
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
        return Objects.equals(m_positions, rec.m_positions)
                && Objects.equals(m_estimate, rec.m_estimate);
    }

    public LocalizerEstimate estimate() {
        return m_estimate;
    }

    public SwerveModulePositions positions() {
        return m_positions;
    }

    public Rotation2d gyroYaw() {
        return m_gyroYaw;
    }

    public NoisyPose2d visionMeasurement() {
        return m_visionMeasurement;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_positions, m_estimate);
    }

    @Override
    public String toString() {
        return "SwerveState [m_estimate=" + m_estimate
                + ", m_wheelPositions=" + m_positions
                + ", m_gyroYaw=" + m_gyroYaw
                + ", m_visionMeasurement=" + m_visionMeasurement + "]";
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

        LocalizerEstimate startEstimate = estimate();
        LocalizerEstimate endEstimate = endValue.estimate();

        // Interpolate the wheel positions.
        SwerveModulePositions startPositions = positions();
        SwerveModulePositions endPositions = endValue.positions();
        SwerveModulePositions wheelLerp = startPositions.interpolate(endPositions, t);

        // Interpolate the gyro measurement.
        Rotation2d gyroLerp = gyroYaw().interpolate(
                endValue.gyroYaw(), t);

        NoisyPose2d measurementLerp = null;
        if (visionMeasurement() != null && endValue.visionMeasurement() != null) {
            measurementLerp = visionMeasurement()
                    .interpolate(endValue.visionMeasurement(), t);
        }

        LocalizerEstimate estimate = startEstimate.interpolate(endEstimate, t);
        return new SwerveState(
                estimate, wheelLerp, gyroLerp, measurementLerp);
    }

}