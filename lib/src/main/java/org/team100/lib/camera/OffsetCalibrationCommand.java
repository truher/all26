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
import org.wpilib.command2.Command;
import org.wpilib.util.struct.StructBuffer;

/**
 * Finds one tag per camera, sets tag to the origin, solves for camera pose,
 * publishes it.
 * 
 * This reads the camera for awhile (30 frames), in order to get enough data for
 * the model to come up with a good estimate.
 * 
 * The tag should be stationary during this time!
 */
public class OffsetCalibrationCommand extends Command {

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
    /**
     * Reader is not final because we make a new one every time the command runs, in
     * order to keep the listener from spinning unnecessarily.
     */
    private CameraReader<BlipWithCorners> m_reader;

    public OffsetCalibrationCommand(LoggerFactory parent) {
        m_log = parent.type(this);
        m_queue = new EnumMap<>(Camera.class);
        m_meanLoggers = new EnumMap<>(Camera.class);
        m_sigmaLoggers = new EnumMap<>(Camera.class);
        m_solver = new GtsamOffsetSolver();
    }

    @Override
    public void initialize() {
        // Forget the previously-read input.
        m_queue.clear();
        // Start polling for new camera input.
        m_reader = new CameraReader<>(
                "vision", "blips_with_corners",
                StructBuffer.create(BlipWithCorners.struct));
    }

    @Override
    public void execute() {
        read();
        solve();
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
                // Note this Transform3d is in camera coordinates, z-forward.
                // TODO: use x-forward instead?
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

    @Override
    public boolean isFinished() {
        // Run as long as the user wants it to run.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop polling when the command ends.
        m_reader.close();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
