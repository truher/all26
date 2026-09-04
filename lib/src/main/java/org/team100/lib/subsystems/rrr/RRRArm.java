package org.team100.lib.subsystems.rrr;

import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.hid.DriverVelocity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.subsystems.rn.PositionSubsystemRn;
import org.team100.lib.subsystems.rrr.commands.MoveJointsManually;
import org.team100.lib.subsystems.rrr.commands.MoveWithProfile;
import org.team100.lib.subsystems.rrr.commands.MoveWithSpline;
import org.team100.lib.subsystems.rrr.commands.MoveWithTrajectorySE2;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;
import org.team100.lib.subsystems.se2.commands.ManualPosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;

public interface RRRArm extends PositionSubsystemSE2, PositionSubsystemRn<N3> {

    RRRConfig getConfig();

    RRRConfig getConfigWithinLimits();

    RRRConfig config(Pose2d p);

    RRRVelocity qdot(RRRConfig q, VelocitySE2 xdot);

    Pose2d pose();

    /**
     * May modify q according to joint limits.
     * See getConfigWithinLimits() to see what actually happened.
     */
    void set(RRRConfig q, RRRVelocity qdot, RRRAcceleration qddot);

    void stop();

    double l1();

    double l2();

    double l3();

    default MoveAndHold moveProfiled(ProfileR1 profile, Pose2d goal) {
        return new MoveWithProfile(this, profile, goal);
    }

    default MoveAndHold moveTrajSE2(LoggerFactory log, Pose2d goal, double speed) {
        return new MoveWithTrajectorySE2(log, this, goal, speed);
    }

    default MoveAndHold moveSplined(VelocitySE2 x0dot, Pose2d x1, VelocitySE2 x1dot) {
        return new MoveWithSpline(this, x0dot, x1, x1dot);
    }

    default Command moveJointsManually(Supplier<DriverVelocity> v) {
        return new MoveJointsManually(this, v);
    }

    default Command movePoseManually(Supplier<DriverVelocity> v) {
        return new ManualPosition(v, this);
    }

}
