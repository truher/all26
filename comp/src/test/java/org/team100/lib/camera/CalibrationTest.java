package org.team100.lib.camera;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.vision.apriltag.AprilTagDetection;
import org.wpilib.vision.apriltag.AprilTagDetector;
import org.wpilib.vision.apriltag.AprilTagDetector.Config;
import org.wpilib.vision.camera.OpenCvLoader;

public class CalibrationTest {
    static {
        try {
            OpenCvLoader.forceLoad();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Test
    void test0() throws IOException {
        // we're just using the java detector for convenience
        try (AprilTagDetector detector = new AprilTagDetector()) {
            detector.addFamily("tag36h11");
            Config conf = new AprilTagDetector.Config();
            conf.numThreads = 2;
            // many tags at low-ish resolution, do not decimate
            conf.quadDecimate = 1.0f;
            detector.setConfig(conf);

            Mat image = loadImage("tangram.png");
            System.out.println(image);

            AprilTagDetection[] results = detector.detect(image);
            assertEquals(94, results.length);
            for (AprilTagDetection r : results) {
                System.out.println(r.getId());
                // System.out.println(Arrays.toString(r.getCorners()));
                assertEquals("tag36h11", r.getFamily());
                // assertEquals(1, r.getId());
                assertEquals(0, r.getHamming());
                double marker_length = 0.0635;
                int marker_grid_width = 10;
                int marker_grid_height = 10;
                double tag_spacing = 0.25;
                Pose3d center = center(
                        marker_length,
                        marker_grid_width,
                        marker_grid_height,
                        tag_spacing,
                        r.getId());
                System.out.println(center);
                List<Translation3d> landmarks = landmarks(marker_length, center);
                System.out.println(landmarks);
            }
        }
    }

    /**
     * Center pose of the given tag id.
     * 
     * In the Tangram image, zero is lower left, one is to the right of that,
     * ten is above zero, etc.
     */
    Pose3d center(
            double marker_length,
            int marker_grid_width,
            int marker_grid_height,
            double tag_spacing,
            int tagId) {
        int col = tagId % marker_grid_width;
        int row = tagId / marker_grid_width;
        double grid_spacing = marker_length * (1 + tag_spacing);
        // the "y" direction here in WPI Pose3d, positive left.
        double y = -1.0 * col * grid_spacing;
        // the "z" direction here in WPI Pose3d, positive up
        double z = row * grid_spacing;
        return new Pose3d(0, y, z, Rotation3d.ZERO);
    }

    List<Translation3d> landmarks(double marker_length, Pose3d center) {
        double H = marker_length / 2;
        // these are in x-forward, y-left, z-up coordinates.
        Transform3d LL = new Transform3d(0, H, -H, Rotation3d.ZERO);
        Transform3d LR = new Transform3d(0, -H, -H, Rotation3d.ZERO);
        Transform3d UR = new Transform3d(0, -H, H, Rotation3d.ZERO);
        Transform3d UL = new Transform3d(0, H, H, Rotation3d.ZERO);
        return List.of(
                center.transformBy(LL).getTranslation(),
                center.transformBy(LR).getTranslation(),
                center.transformBy(UR).getTranslation(),
                center.transformBy(UL).getTranslation());
    }

    // from wpilib
    public Mat loadImage(String resource) throws IOException {
        Mat encoded;
        try (InputStream is = getClass().getResource(resource).openStream()) {
            try (ByteArrayOutputStream os = new ByteArrayOutputStream(is.available())) {
                byte[] buffer = new byte[4096];
                int bytesRead;
                while ((bytesRead = is.read(buffer)) != -1) {
                    os.write(buffer, 0, bytesRead);
                }
                encoded = new Mat(1, os.size(), CvType.CV_8U);
                encoded.put(0, 0, os.toByteArray());
            }
        }
        Mat image = Imgcodecs.imdecode(encoded, Imgcodecs.IMREAD_COLOR);
        encoded.release();
        Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2GRAY);
        return image;
    }

}
