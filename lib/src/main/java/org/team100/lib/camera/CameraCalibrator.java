package org.team100.lib.camera;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import org.team100.lib.localization.network.BlipWithCorners;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.network.CameraReader;
import org.team100.lib.util.NamedChooser;
import org.wpilib.util.struct.StructBuffer;

/**
 * Calibrate camera intrinsics and distortion using the tag-array target.
 * 
 * Should run in TimedRobot.addPeriodic().
 */
public class CameraCalibrator {
    private final Map<Camera, Map<Integer, List<BlipWithCorners>>> m_queue;

    private final NamedChooser<Boolean> m_enable;

    private CameraReader<BlipWithCorners> m_reader;

    public CameraCalibrator(LoggerFactory parent) {
                m_queue = new EnumMap<>(Camera.class);

        // this is the same as the experiment setup; we need the "onchange".
        String name = "Solve Camera Calibration";
        m_enable = new NamedChooser<>(name);
        m_enable.addDefault(name + " OFF", false);
        m_enable.add(name + " ON", true);
        m_enable.onChange(this::enable);
    }

    public void execute() {
        System.out.println("CAMERA CALIBRATOR");
        Boolean enabled = m_enable.getSelected();
        if (enabled != null && enabled) {
            // read();
            // solve();
        }
    }

    private void enable(Boolean enabled) {
        if (enabled != null && enabled) {
            // Forget the previously-read input.
            m_queue.clear();
            // Start polling for new camera input.
            m_reader = new CameraReader<>(
                    "vision",
                    "blips_with_corners",
                    StructBuffer.create(BlipWithCorners.struct),
                    50);
        } else {
            // Stop polling when the command ends.
            m_reader.close();
            m_reader = null;
        }
    }

    private void read() {
        List<CameraReader.Record<BlipWithCorners>> records = m_reader.getRecords();

    }

}