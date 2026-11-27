package org.team100.lib.hid;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.geometry.se2.VelocitySE2;
import org.wpilib.driverstation.Alliance;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.util.MathUtil;

/**
 * Inverts driver input depending on alliance.
 * 
 * Previously to 2027, we inverted the field map, and kept driver input the
 * same.
 * 
 * For 2027, we'll keep the field map the same, and invert driver input.
 */
public class FieldRelativeControl {

    public static VelocitySE2 forAlliance(
            VelocitySE2 x, Supplier<Optional<Alliance>> s) {
        if (RobotBase.isSimulation()) {
            // The simulator control is different, leave it alone.
            return x;
        }
        Optional<Alliance> o = s.get();
        if (o.isEmpty()) {
            // Can't tell what to do, so do nothing.
            return x;
        }
        Alliance a = o.get();
        return switch (a) {
            case RED -> flip(x);
            case BLUE -> x;
            default -> x;
        };
    }

    /** Rotate 180 degrees */
    private static VelocitySE2 flip(VelocitySE2 x) {
        return new VelocitySE2(-x.x(), -x.y(), MathUtil.angleModulus(x.theta() + Math.PI));
    }

}
