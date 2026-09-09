package org.team100.lib.sensor.position.absolute;

import org.team100.lib.util.CanId;
import org.wpilib.math.util.MathUtil;

import com.reduxrobotics.sensors.canandmag.Canandmag;

public class ReduxPositionSensor implements RotaryPositionSensor {
    private final Canandmag encoder;

    public ReduxPositionSensor(CanId id) {
        encoder = new Canandmag(id.id);
    }

    @Override
    public double getWrappedPositionRad() {
        return MathUtil.angleModulus(2 * Math.PI * encoder.getPosition());
    }

    @Override
    public double getUnwrappedPositionRad() {
        return 2 * Math.PI * encoder.getPosition();
    }

    @Override
    public double getVelocityRad_S() {
        return 2 * Math.PI * encoder.getVelocity();
    }

    @Override
    public double getAccelerationRad_S2() {
        return 0;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void close() {
        encoder.close();
    }

}
