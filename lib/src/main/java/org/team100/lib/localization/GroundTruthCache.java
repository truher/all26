package org.team100.lib.localization;

import java.util.function.Supplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.SideEffect;
import org.team100.lib.coherence.Takt;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;

/**
 * Run the ground truth odometry updater at the right time.
 * 
 * Similar to FreshSwerveEstimate.
 */
public class GroundTruthCache {
    private final SwerveHistory m_history;
    private final OdometryEstimator m_updater;
    private final Gyro m_gyro;
    private final Supplier<SwerveModulePositions> m_positions;
    private final SideEffect m_cache;

    public GroundTruthCache(
            OdometryEstimator odometry,
            Gyro gyro,
            Supplier<SwerveModulePositions> positions,
            SwerveHistory history) {
        m_history = history;
        m_updater = odometry;
        m_gyro = gyro;
        m_positions = positions;
        m_cache = Cache.ofSideEffect(this::update);
    }

    void update() {
        double timestamp = Takt.get();
        SwerveState s = m_updater.estimate(
                timestamp,
                m_gyro.getYawNWU(),
                m_positions.get());
        if (s != null)
            m_history.put(timestamp, s);
    }

    public StateSE2 apply(double timestampS) {
        // m_odometry.run();
        m_cache.run();
        return m_history.get(timestampS);
    }

}