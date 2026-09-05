package org.team100.lib.subsystems.r2;

import org.team100.lib.state.ControlR2;
import org.team100.lib.state.StateR2;

/**
 * A planar subsystem for position only, not rotation.
 */
public interface PositionSubsystemR2 extends SubsystemR2 {
    /**
     * Position, velocity, and acceleration in R2.
     * 
     * Subsystems are expected to compute the (generalized)
     * force required to meet this setpoint, using the dynamics
     * of the mechanism.
     * 
     * @param setpoint for the next timestamp
     */
    StateR2 set(ControlR2 setpoint);
}
