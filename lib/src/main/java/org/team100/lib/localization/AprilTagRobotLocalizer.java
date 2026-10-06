package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.localization.NudgingVisionUpdater.VisionMeasurement;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.network.CameraReader;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Extracts robot pose estimates from camera observations of AprilTags.
 * 
 * Note this class depends only on the state *history*, not on the coherent sate
 * *estimate*. The camera input doesn't require fresh odometry, it modifies the
 * past (and replays up to the present).
 */
public class AprilTagRobotLocalizer {
    private static final boolean DEBUG = false;

    /** Discard results further than this from the previous one. */
    private static final double VISION_CHANGE_TOLERANCE_M = 0.25;
    private final CameraReader<Blip> m_reader;
    final AprilTagTranslator m_translator;
    private final VisionUpdater m_visionUpdater;

    // LOGGERS

    private final Pose2dLogger m_log_pose;

    /**
     * The difference between the current instant and the instant of the blip,
     * including our magic correction, i.e. this is the time we look up in the pose
     * buffer.
     */
    private final DoubleLogger m_log_lag;

    /**
     * Remember the previous vision-based pose estimate, so we can measure the
     * distance between consecutive updates, and ignore too-far updates.
     */
    private Pose2d m_prevPose;

    /**
     * @param parent        logger
     * @param layout        map of apriltags
     * @param history       f(timestamp) = swerve state
     * @param visionUpdater mutates history
     */
    public AprilTagRobotLocalizer(
            LoggerFactory parent,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            VisionUpdater visionUpdater,
            Supplier<Optional<Alliance>> alliance) {
        LoggerFactory log = parent.type(this);
        m_reader = new CameraReader<>("vision", "blips",
                StructBuffer.create(Blip.struct));
        m_translator = new AprilTagTranslator(log, layout, alliance);

        m_visionUpdater = visionUpdater;

        m_log_pose = log.pose2dLogger(Level.TRACE, "pose");
        m_log_lag = log.doubleLogger(Level.TRACE, "lag");

    }

    public void update() {
        List<VisionMeasurement> measurements = read();
        for (VisionMeasurement m : measurements) {
            m_log_lag.log(() -> Takt.get() - m.timestamp());
            Pose2d pose = m.noisyMeasurement().pose();
            m_log_pose.log(() -> pose);

            //////////////////////////////////////////////////////////////////
            ///
            /// Should we use this update?
            ///
            if (Experiments.INSTANCE.enabled(Experiment.IgnoreVision)) {
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

            if (Metrics.translationalDistance(m_prevPose, pose) > VISION_CHANGE_TOLERANCE_M) {
                // No, the new estimate is too far from the previous one.
                m_prevPose = pose;
                if (DEBUG)
                    System.out.printf("New estimate %s is too far away from old %s.", pose, m_prevPose);
                continue;
            }

            ///
            /// Yes, we should use this update.
            ///
            //////////////////////////////////////////////////////////////////
            m_prevPose = pose;
            m_visionUpdater.put(m.timestamp(), m.noisyMeasurement());
        }
    }

    public List<VisionMeasurement> read() {
        List<CameraReader.Record<Blip>> records = m_reader.getRecords();
        List<VisionMeasurement> measurements = new ArrayList<>();
        for (CameraReader.Record<Blip> r : records) {
            measurements.addAll(m_translator.perValue(r.camera(), r.values()));
        }
        return measurements;
    }

    /**
     * Tags outside this radius are ignored.
     */
    public void setHeedRadiusM(double heedRadiusM) {
        m_translator.setHeedRadiusM(heedRadiusM);
    }

}