package org.team100.lib.util;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.wpilib.vision.camera.OpenCvLoader;

public class ImgUtil {
    static {
        try {
            OpenCvLoader.forceLoad();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Pitch
     * 
     * @param theta yaw rotation
     * @param d     distance
     */
    public static Mat xrot(double theta, double d) {
        Mat T = new Mat(4, 4, CvType.CV_64FC1);
        T.put(0, 0, //
                1, 0, 0, 0, //
                0, Math.cos(theta), -Math.sin(theta), 0, //
                0, Math.sin(theta), Math.cos(theta), d, //
                0, 0, 0, 1);
        return T;
    }

    public static Mat t(double x, double y, double z) {
        Mat T = new Mat(4, 4, CvType.CV_64FC1);
        T.put(0, 0, //
                1, 0, 0, x, //
                0, 1, 0, y, //
                0, 0, 1, z, //
                0, 0, 0, 1);
        return T;
    }

    /**
     * Yaw
     * 
     * @param theta yaw rotation
     * @param d     distance
     */
    public static Mat yrot(double theta, double d) {
        Mat T = new Mat(4, 4, CvType.CV_64FC1);
        T.put(0, 0, //
                Math.cos(theta), 0, Math.sin(theta), 0, //
                0, 1, 0, 0, //
                -Math.sin(theta), 0, Math.cos(theta), d, //
                0, 0, 0, 1);
        return T;
    }

    /**
     * Transform via matrix
     * 
     * @param T   homogeneous transformation matrix
     * @param img input image
     * @param f   focal length
     */
    public static Mat transform(Mat T, Mat img, double f) {
        Size size = img.size();
        double cx = size.width / 2;
        double cy = size.height / 2;
        Mat warped_img = new Mat();

        // to 3d
        Mat A1 = new Mat(4, 3, CvType.CV_64FC1);
        A1.put(0, 0, //
                1, 0, -cx, //
                0, 1, -cy, //
                0, 0, 0, //
                0, 0, 1);
        // projection to 2d
        Mat A2 = new Mat(3, 4, CvType.CV_64FC1);
        A2.put(0, 0, //
                f, 0, cx, 0, //
                0, f, cy, 0, //
                0, 0, 1, 0);
        Mat M = A2.matMul(T).matMul(A1);
        // System.out.printf("M\n%s\n", M.dump());
        Imgproc.warpPerspective(img, warped_img, M, img.size(),
                Imgproc.INTER_LINEAR, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));
        return warped_img;
    }

    // from wpilib
    public static Mat loadImage(Class<?> clazz, String resource) {
        URL res = clazz.getResource(resource);
        if (res == null)
            throw new IllegalArgumentException(String.format("no file at %s", resource));
        try (InputStream is = res.openStream()) {
            try (ByteArrayOutputStream os = new ByteArrayOutputStream(is.available())) {
                byte[] buffer = new byte[4096];
                int bytesRead;
                while ((bytesRead = is.read(buffer)) != -1) {
                    os.write(buffer, 0, bytesRead);
                }
                Mat encoded = new Mat(1, os.size(), CvType.CV_8U);
                encoded.put(0, 0, os.toByteArray());
                Mat image = Imgcodecs.imdecode(encoded, Imgcodecs.IMREAD_COLOR);
                encoded.release();
                Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2GRAY);
                return image;
            }
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }
}
