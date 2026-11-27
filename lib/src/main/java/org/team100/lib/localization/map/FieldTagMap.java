package org.team100.lib.localization.map;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.system.Filesystem;

/**
 * The WPILib JSON tag file, and the wrapper, AprilTagFieldLayout, define tag
 * rotation with respect to the "out of the page" normal, which is the opposite
 * of the AprilTags convention to use the "into the page" normal.
 * 
 * This wrapper "fixes" the orientations so they match the Apriltag convention,
 * and thus match the result of camera pose estimates.
 * 
 * This class used to "flip" the map depending on the alliance, but we don't do
 * that anymore.
 */
public class FieldTagMap {
    private static final String FILENAME = "2026-rebuilt-andymark.json";

    /** Inverts yaw. */
    private static final Transform3d FIX = new Transform3d(
            new Translation3d(),
            new Rotation3d(0, 0, Math.PI));

    /** Key: tag id */
    private final Map<Integer, FieldTag> m_tags;
    private final double m_length;
    private final double m_width;

    public FieldTagMap() throws IOException {
        this(FILENAME);
    }

    /** For testing only */
    public FieldTagMap(String filename) throws IOException {
        Path path = Filesystem.getDeployDirectory().toPath().resolve(filename);
        Field field = Field.loadFromFile(path);
        m_length = field.length;
        m_width = field.width;
        List<FieldTag> tagList = field.getTags();
        if (tagList == null)
            throw new IllegalArgumentException("null tags for filename " + filename);
        m_tags = new HashMap<>();
        for (FieldTag t : tagList) {
            // correct the tag yaws all at once.
            m_tags.put(t.getID(), new FieldTag(t.getID(), t.getPose().transformBy(FIX)));
        }
    }

    /** Traps the IO exception. */
    public static FieldTagMap getLayout() {
        try {
            return new FieldTagMap();
        } catch (IOException e) {
            throw new IllegalStateException("Could not read Apriltag layout file", e);
        }
    }

    /** Pose of the given tag id, if present, with correct yaw. */
    public Optional<Pose3d> getTagPose(int id) {
        FieldTag t = m_tags.get(id);
        if (t == null)
            return Optional.empty();
        return Optional.of(t.getPose());
    }

    /** All tags in the map. */
    public Collection<FieldTag> all() {
        return m_tags.values();
    }

    public double length() {
        return m_length;
    }

    public double width() {
        return m_width;
    }

    public Translation2d extent() {
        return new Translation2d(m_length, m_width);
    }
}
