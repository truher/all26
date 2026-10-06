package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.network.CameraReader;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Uses the corners of observed AprilTags to derive the robot pose.
 * 
 * Note this class depends only on the state *history*, not on the coherent sate
 * *estimate*. The camera input doesn't require fresh odometry, it modifies the
 * past (and replays up to the present).
 */
public class AprilTagCornerRobotLocalizer {
    private static final boolean DEBUG = false;
    /** Discard results further than this from the previous one. */
    private static final double VISION_CHANGE_TOLERANCE_M = 0.25;

    private final CameraReader<BlipWithCorners> m_reader;
    final AprilTagCornerTranslator m_translator;
    private final VisionEstimator m_visionUpdater;
    private final DoubleConsumer m_replayer;
    private final SwerveHistory m_history;

    private final Pose2dLogger m_log_pose;
    /**
     * The difference between the current instant and the instant of the blip,
     * i.e. this is the time we look up in the pose buffer.
     */
    private final DoubleLogger m_log_lag;

    /**
     * Remember the previous vision-based pose estimate, so we can measure the
     * distance between consecutive updates, and ignore too-far updates.
     */
    private Pose2d m_prevPose;

    public AprilTagCornerRobotLocalizer(
            LoggerFactory parent,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            VisionEstimator visionUpdater,
            DoubleConsumer replayer,
            SwerveHistory history,
            Supplier<Optional<Alliance>> alliance) {
        LoggerFactory log = parent.type(this);
        m_reader = new CameraReader<>("vision", "blips_with_corners",
                StructBuffer.create(BlipWithCorners.struct));
        m_translator = new AprilTagCornerTranslator(log, layout, alliance);
        m_visionUpdater = visionUpdater;
        m_replayer = replayer;
        m_history = history;
        m_log_pose = log.pose2dLogger(Level.TRACE, "pose");
        m_log_lag = log.doubleLogger(Level.TRACE, "lag");
    }

    public void update() {
        List<VisionMeasurement> filteredMeasurements = filteredRead();
        consumeMeasurement(filteredMeasurements);
    }

    private void consumeMeasurement(List<VisionMeasurement> filteredMeasurements) {
        for (VisionMeasurement m : filteredMeasurements) {
            SwerveState state = m_visionUpdater.estimate(m.timestamp(), m.noisyMeasurement());
            if (state != null && !m_history.tooOld(m.timestamp())) {
                m_history.put(m.timestamp(), state);
                m_replayer.accept(m.timestamp());
            }
        }
    }

    private List<VisionMeasurement> filteredRead() {
        List<VisionMeasurement> measurements = read();
        List<VisionMeasurement> filteredMeasurements = new ArrayList<>();
        for (VisionMeasurement m : measurements) {
            m_log_lag.log(() -> Takt.get() - m.timestamp());
            Pose2d pose = m.noisyMeasurement().pose();
            m_log_pose.log(() -> pose);

            //////////////////////////////////////////////////////////////////
            ///
            /// Should we use this update?
            ///
            if (Experiments.INSTANCE.enabled(Experiment.IgnoreVision)) {
                // No, vision is off.
                if (DEBUG)
                    System.out.println("Drop update, vision is off.");
                continue;
            }
            if (m_prevPose == null) {
                // No, we need another nearby fix to believe either one.
                m_prevPose = pose;
                if (DEBUG)
                    System.out.println("Need confirmation.");
                continue;
            }
            double distanceFromPrev = Metrics.translationalDistance(m_prevPose, pose);
            if (distanceFromPrev > VISION_CHANGE_TOLERANCE_M) {
                // No, the new estimate is too far from the previous one.
                m_prevPose = pose;
                if (DEBUG)
                    System.out.println("New estimate is too far away.");
                continue;
            }
            ///
            /// Yes, we should use this update.
            ///
            //////////////////////////////////////////////////////////////////
            m_prevPose = pose;
            filteredMeasurements.add(m);
        }
        return filteredMeasurements;
    }

    /** Read all pending input and return a list of measurements */
    public List<VisionMeasurement> read() {
        // camera inputs
        List<CameraReader.Record<BlipWithCorners>> records = m_reader.getRecords();
        List<VisionMeasurement> measurements = new ArrayList<>();
        for (CameraReader.Record<BlipWithCorners> r : records) {
            measurements.addAll(m_translator.convert(r.camera(), r.values()));
        }
        return measurements;
    }

    /**
     * Tags outside this radius are ignored.
     * 
     * TODO: remove this, the noise model should take care of it.
     */
    void setHeedRadiusM(double heedRadiusM) {
        m_translator.setHeedRadiusM(heedRadiusM);
    }

}