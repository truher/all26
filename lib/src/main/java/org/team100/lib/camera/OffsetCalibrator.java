package org.team100.lib.camera;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.team100.lib.localization.network.BlipWithCorners;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.Transform3dLogger;
import org.team100.lib.network.CameraReader;
import org.team100.lib.uncertainty.NoisyTransform3d;
import org.team100.lib.util.NamedChooser;
import org.wpilib.util.struct.StructBuffer;

/**
 * Offset calibration worker, should run in TimedRobot.addPeriodic().
 * 
 * Set a tag in a known location relative to the robot, and then turn this on,
 * and it will log the transform from the camera pose to the tag pose. Use that
 * transform in
 * 
 * Finds one tag per camera, sets tag to the origin, solves for camera pose,
 * publishes it.
 * 
 * This reads the camera for awhile (30 frames), in order to get enough data for
 * the model to come up with a good estimate.
 * 
 * The tag should be stationary during this time!
 */
public class OffsetCalibrator {

    private final LoggerFactory m_log;

    /**
     * Accumulated input for the batch solver.
     */
    private final Map<Camera, Map<Integer, List<BlipWithCorners>>> m_queue;

    /**
     * Accumulated loggers we use.
     */
    private final Map<Camera, Map<Integer, Transform3dLogger>> m_meanLoggers;
    private final Map<Camera, Map<Integer, DoubleArrayLogger>> m_sigmaLoggers;

    private final GtsamOffsetSolver m_solver;

    private final NamedChooser<Boolean> m_enable;
    /**
     * Reader is not final because we make a new one every time the command runs, in
     * order to keep the listener from spinning unnecessarily.
     */
    private CameraReader<BlipWithCorners> m_reader;

    public OffsetCalibrator(LoggerFactory parent) {
        m_log = parent.type(this);
        m_queue = new EnumMap<>(Camera.class);
        m_meanLoggers = new EnumMap<>(Camera.class);
        m_sigmaLoggers = new EnumMap<>(Camera.class);
        m_solver = new GtsamOffsetSolver();
        // this is the same as the experiment setup; we need the "onchange".
        String name = "Solve Camera->Tag";
        m_enable = new NamedChooser<>(name);
        m_enable.addDefault(name + " OFF", false);
        m_enable.add(name + " ON", true);
        m_enable.onChange(this::enable);
    }

    /**
     * This should be called periodically, e.g. in TimedRobot.addPeriodic().
     */
    public void execute() {
        Boolean enabled = m_enable.getSelected();
        if (enabled == null || !enabled)
            return;
        read();
        solve();
    }

    /**
     * Updated by the chooser.
     */
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
            // Stop polling.
            m_reader.close();
            m_reader = null;
        }
    }

    /**
     * Read input and add it to the appropriate queue.
     */
    private void read() {
        List<CameraReader.Record<BlipWithCorners>> records = m_reader.getRecords();
        for (CameraReader.Record<BlipWithCorners> record : records) {
            Camera camera = record.camera();
            Map<Integer, List<BlipWithCorners>> camQueue = m_queue.computeIfAbsent(
                    camera, (c) -> new HashMap<>());
            BlipWithCorners[] blips = record.values();
            for (int i = 0; i < blips.length; ++i) {
                BlipWithCorners blip = blips[i];
                int tagId = blip.getId();
                List<BlipWithCorners> camTagList = camQueue.computeIfAbsent(
                        tagId, x -> new ArrayList<>());
                camTagList.add(blip);
            }
        }
    }

    /**
     * For any queues that are full enough, run the solver and publish the output.
     */
    private void solve() {
        for (Map.Entry<Camera, Map<Integer, List<BlipWithCorners>>> perCam : m_queue.entrySet()) {
            Camera camera = perCam.getKey();
            for (Map.Entry<Integer, List<BlipWithCorners>> perTag : perCam.getValue().entrySet()) {
                int tagId = perTag.getKey();
                List<BlipWithCorners> blips = perTag.getValue();
                if (blips.size() < 30)
                    continue;
                // There are enough blips to run the solver.
                NoisyTransform3d result = m_solver.solve(camera, blips);
                blips.clear();
                // Log the results
                LoggerFactory f = m_log.name(camera.name()).name(Integer.toString(tagId));
                Map<Integer, Transform3dLogger> camT = m_meanLoggers.computeIfAbsent(
                        camera, (c) -> new HashMap<>());
                Transform3dLogger tl = camT.computeIfAbsent(
                        tagId, x -> f.transform3dLogger(Level.COMP, "transform"));
                tl.log(() -> result.transform());
                Map<Integer, DoubleArrayLogger> camS = m_sigmaLoggers.computeIfAbsent(
                        camera, (c) -> new HashMap<>());
                DoubleArrayLogger sl = camS.computeIfAbsent(
                        tagId, x -> f.doubleArrayLogger(Level.COMP, "sigma"));
                sl.log(() -> result.sigma());
            }
        }
    }

}
