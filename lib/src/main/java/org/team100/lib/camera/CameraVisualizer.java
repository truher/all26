package org.team100.lib.camera;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import org.team100.lib.localization.network.BlipWithCorners;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.StructArrayLogger;
import org.team100.lib.logging.primitive.NTPrimitiveLogger;
import org.team100.lib.logging.primitive.PrimitiveLogger;
import org.team100.lib.network.CameraReader;
import org.team100.lib.util.NamedChooser;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.util.struct.StructBuffer;

/**
 * Make a Field2d for each camera, and show the tag corners each camera sees.
 * 
 * Registers itself with LogPoller, no need to retain another reference.
 */
public class CameraVisualizer {
    private final PrimitiveLogger m_ntLogger;
    private final Map<Camera, List<Pose2d>> m_poses;
    private final Map<Camera, StructArrayLogger<Pose2d>> m_loggers;
    private final NamedChooser<Boolean> m_enable;
    private CameraReader<BlipWithCorners> m_reader;

    public CameraVisualizer() {
        m_ntLogger = new NTPrimitiveLogger();
        m_poses = new EnumMap<>(Camera.class);
        m_loggers = new EnumMap<>(Camera.class);
        // this is the same as the experiment setup; we need the "onchange".
        String name = "Show Camera Frames";
        m_enable = new NamedChooser<>(name);
        m_enable.addDefault(name + " OFF", false);
        m_enable.add(name + " ON", true);
        m_enable.onChange(this::enable);
        LogPoller.register(this::log);
    }

    private void enable(Boolean enabled) {
        if (enabled != null && enabled) {
            // Start polling for new camera input.
            m_reader = new CameraReader<>(
                    "vision",
                    "blips_with_corners",
                    StructBuffer.create(BlipWithCorners.struct));
        } else {
            // Stop polling.
            m_reader.close();
            m_reader = null;
            clear();
            write();
        }
    }

    private void log() {
        Boolean enabled = m_enable.getSelected();
        if (enabled == null || !enabled)
            return;
        clear();
        read();
        write();
    }

    /**
     * Clear all buffers so that each frame starts empty.
     */
    private void clear() {
        for (List<Pose2d> poses : m_poses.values()) {
            poses.clear();
        }
    }

    /**
     * Consume all the pending input and add it to the list for each camera.
     */
    private void read() {
        for (CameraReader.Record<BlipWithCorners> record : m_reader.getRecords()) {
            Camera cam = record.camera();
            List<Pose2d> poses = m_poses.computeIfAbsent(
                    cam, x -> new ArrayList<>());
            for (BlipWithCorners blip : record.values()) {
                poses.add(pose(cam, blip.getX0(), blip.getY0()));
                poses.add(pose(cam, blip.getX1(), blip.getY1()));
                poses.add(pose(cam, blip.getX2(), blip.getY2()));
                poses.add(pose(cam, blip.getX3(), blip.getY3()));
            }
        }
    }

    /**
     * Write out the corner pixel values to the widget for each camera.
     */
    private void write() {
        for (Map.Entry<Camera, List<Pose2d>> entry : m_poses.entrySet()) {
            m_loggers.computeIfAbsent(entry.getKey(), this::logger)
                    .log(() -> entry.getValue().toArray(Pose2d[]::new));
        }
    }

    /**
     * Make a logger for the given camera.
     */
    private StructArrayLogger<Pose2d> logger(Camera camera) {
        // level here is "selected level" not "logger level"
        LoggerFactory fieldLogger = new LoggerFactory(
                () -> Level.TRACE, "Telemetry/" + camera.name(), m_ntLogger);
        fieldLogger.stringLogger(Level.COMP, ".type").log(() -> "Field2d");
        return fieldLogger.structArrayLogger(Level.COMP, "corners", Pose2d.struct);
    }

    /**
     * Pose for pixel values, normalized using the focal length.
     * 
     * Inverts the vertical coordinate, since the field widget is x-right, y-up
     * and the camera is x-right y-down. Does not subtract the intrinsic center,
     * because the field widget only uses positive numbers.
     * 
     * @param u pixel "u", camera horizontal, positive right
     * @param v pixel "v" , camera vertical, positive down
     */
    private Pose2d pose(Camera camera, double u, double v) {
        Intrinsic K = Intrinsic.get(camera);
        double x = u / K.fx();
        double y = (2 * K.cy() - v) / K.fy();
        return new Pose2d(x, y, Rotation2d.ZERO);
    }

}
