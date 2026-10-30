package org.team100.lib.localization;

import java.util.Map.Entry;
import java.util.SortedMap;

import org.team100.lib.state.StateSE2;
import org.team100.lib.util.TimeInterpolatableBuffer100;

/**
 * History is just a container, in fact the implementation is little more than a
 * wrapper around TimeInterpolatableBuffer.
 * 
 * The history always has *something* in it, even the initial zero pose.
 * 
 * The buffer only needs to be long enough to catch stale-but-still-helpful
 * vision updates.
 * 
 * The current Raspberry Pi cameras seem to be able to provide frames to RoboRIO
 * code with about 75-100 ms latency. There will never be a vision update
 * older than about 200 ms.
 */
public class SwerveHistory {
    private final double m_bufferDuration;

    // these two maps need to have the same keys all the time
    // ... which seems bad.

    private final TimeInterpolatableBuffer100<OdoGyroMeasurement> m_measurements;
    private final TimeInterpolatableBuffer100<LocalizerEstimate> m_estimates;

    public SwerveHistory(
            double bufferDuration,
            double initialTimestamp,
            OdoGyroMeasurement initialMeasurement,
            LocalizerEstimate initialEstimate) {
        m_bufferDuration = bufferDuration;
        m_estimates = new TimeInterpolatableBuffer100<>(
                bufferDuration, initialTimestamp, initialEstimate);
        m_measurements = new TimeInterpolatableBuffer100<>(
                bufferDuration, initialTimestamp, initialMeasurement);
    }

    public ImmutableSwerveHistory immutableCopy() {
        return new ImmutableSwerveHistory(m_estimates.entrySet(), m_bufferDuration);
    }

    /**
     * Sample the state estimate buffer. May return an entry exactly, if the
     * timestamp matches a known timestamp exactly. Otherwise returns an
     * interpolated value.
     */
    public StateSE2 get(double timestampSeconds) {
        return m_estimates.get(timestampSeconds).state();
    }

    /** Empty the buffer and add the given measurements. */
    public void reset(
            OdoGyroMeasurement measurement,
            LocalizerEstimate estimate,
            double timestampSeconds) {
        m_measurements.reset(timestampSeconds, measurement);
        m_estimates.reset(timestampSeconds, estimate);
    }

    //////////////////////////////////////////////////
    //
    // Methods below are for history maintenance and testing.

    /**
     * @param timestamp   time, seconds
     * @param estimate    from estimator
     * @param measurement from odometry/gyro
     */
    public void put(
            double timestamp,
            LocalizerEstimate estimate,
            OdoGyroMeasurement measurement) {
        m_estimates.put(timestamp, estimate);
        m_measurements.put(timestamp, measurement);
    }

    /** Entry for time strictly before timestamp. Never interpolated. */
    public Entry<Double, OdoGyroMeasurement> measurementLowerEntry(double timestamp) {
        return m_measurements.lowerEntry(timestamp);
    }

    public Entry<Double, LocalizerEstimate> estimateLowerEntry(double timestamp) {
        return m_estimates.lowerEntry(timestamp);
    }

    /**
     * Sample the buffer at the given time. May return an entry exactly, if the
     * timestamp matches a known timestamp exactly. Otherwise returns an
     * interpolated value. Don't use this method outside this package.
     */
    public OdoGyroMeasurement getOdoGyroMeasurement(double timestamp) {
        return m_measurements.get(timestamp);
    }

    public LocalizerEstimate getLocalizerEstimate(double timestamp) {
        return m_estimates.get(timestamp);
    }

    public boolean tooOld(double timestamp) {
        return m_measurements.tooOld(timestamp);
    }

    /** strictly later than the timestamp. */
    public SortedMap<Double, OdoGyroMeasurement> measurementTailMap(double timestamp) {
        return m_measurements.tailMap(timestamp, false);
    }

    int size() {
        return m_measurements.size();
    }

    double lastKey() {
        return m_measurements.lastKey();
    }

    /** Print the buffer contents. */
    public void dump() {
        m_estimates.dump();
        m_measurements.dump();
    }

}
