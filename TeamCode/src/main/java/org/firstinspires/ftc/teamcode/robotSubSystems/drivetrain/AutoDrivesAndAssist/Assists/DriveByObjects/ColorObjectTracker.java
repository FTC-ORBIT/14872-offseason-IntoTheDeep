package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.Assists.DriveByObjects;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

public class ColorObjectTracker implements VisionProcessor {

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    private final Scalar lowerYellow = new Scalar(20, 100, 100);
    private final Scalar upperYellow = new Scalar(40, 255, 255);

    private final Scalar lowerBlue = new Scalar(100, 150, 0);
    private final Scalar upperBlue = new Scalar(140, 255, 255);

    private final Scalar lowerRed = new Scalar(0, 100, 100);
    private final Scalar upperRed = new Scalar(10, 255, 255);

    Scalar highUpperRed = new Scalar(160, 100, 20); // Wraps around Color Wheel
    Scalar highLowerRed = new Scalar(180, 255, 255);

    private static double distance = 0;
    private static double angle = 0;
    private static String detectedColor = "";
    private static double yaw = 0;
    private static boolean objectFound = false;
    Point closestBlobCenter = null;

    private double calculateDistance(double area) {
        return Math.sqrt(area * 10); // inches
    }

    private double calculateAngle(Point center) {
        double x = center.x - (double) CAMERA_WIDTH / 2;
        double y = center.y - (double) CAMERA_HEIGHT / 2;
        return Math.toDegrees(Math.atan2(x, y));
    }

    private double calculateYaw(Point center) {
        double x = center.x - (double) CAMERA_WIDTH / 2;
        double y = center.y - (double) CAMERA_HEIGHT / 2;
        return Math.toDegrees(Math.atan2(y, x));
    }

    public float calcDirection() {
        if (closestBlobCenter != null) {
            return (float) Math.atan2(closestBlobCenter.y - CAMERA_HEIGHT / 2.0, closestBlobCenter.x - CAMERA_WIDTH / 2.0);
        }
        return 0;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(hsvFrame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        Mat thresholdFrame = new Mat();
        Mat lowRedMat = new Mat();
        Mat highRedMat = new Mat();
        closestBlobCenter = null;
        double maxArea = 0;

        Core.inRange(hsvFrame, lowerYellow, upperYellow, thresholdFrame);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresholdFrame, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        findLargestBlob(contours, maxArea);

        if (GlobalData.allianceColor) {
            Core.inRange(hsvFrame, lowerBlue, upperBlue, thresholdFrame);
        } else {
            Core.inRange(hsvFrame, lowerRed, upperRed, lowRedMat);
            Core.inRange(hsvFrame, highLowerRed, highUpperRed, highRedMat);
            Core.bitwise_or(lowRedMat, highRedMat, hsvFrame);
        }

        contours.clear();
        hierarchy.release();
        hierarchy = new Mat();
        Imgproc.findContours(thresholdFrame, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        findLargestBlob(contours, maxArea);

        if (closestBlobCenter != null) {
            distance = calculateDistance(maxArea);
            angle = calculateAngle(closestBlobCenter);
            yaw = calculateYaw(closestBlobCenter);
            objectFound = true;
        } else {
            distance = 0;
            angle = 0;
            yaw = 0;
            objectFound = false;
        }

        return frame;
    }

    private void findLargestBlob(List<MatOfPoint> contours, double maxArea) {
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                Rect boundingRect = Imgproc.boundingRect(contour);
                closestBlobCenter = new Point(boundingRect.x + (double) boundingRect.width / 2, boundingRect.y + (double) boundingRect.height / 2);
            }
        }
    }

    public float getDistance() {
        return (float) distance;
    }

    public float getAngle() {
        return (float) angle;
    }

    public static String getDetectedColor() {
        return detectedColor;
    }

    public float getYaw() {
        return (float) yaw;
    }

    public  boolean isObjectFound() {
        return objectFound;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
}
