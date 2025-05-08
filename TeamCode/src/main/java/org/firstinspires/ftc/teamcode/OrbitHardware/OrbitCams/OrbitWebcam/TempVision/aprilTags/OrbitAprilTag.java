package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitCams.OrbitWebcam.TempVision.aprilTags;

import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose3D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;

public class OrbitAprilTag {

    private static final Pose3D camPose = new Pose3D(0, 0, 0, 0, 0, 0); // TODO => tune!

    public static Vector getPositionToTag(final AprilTagDetection detection) {
        final float xDist = (float) detection.metadata.distanceUnit.toInches(detection.ftcPose.x);
        final float yDist = (float) detection.metadata.distanceUnit.toInches(detection.ftcPose.y);
        return new Vector(xDist, yDist).subtract(camPose.translation.xy());
    }

    public static Vector3D get3DPositionToTag(final AprilTagDetection detection) {
        return new Vector3D(
                (float) detection.metadata.distanceUnit.toInches(detection.ftcPose.x),
                (float) detection.metadata.distanceUnit.toInches(detection.ftcPose.y),
                (float) detection.metadata.distanceUnit.toInches(detection.ftcPose.z)
        ).subtract(camPose.translation);
    }

    public static float getHeadingToTag(final AprilTagDetection detection) {
        return (float) detection.ftcPose.yaw - camPose.rotation.z;
    }

    public static Pose2D getDeltaPoseToTag(final AprilTagDetection detection) {
        return new Pose2D(getPositionToTag(detection), getHeadingToTag(detection));
    }

    public static Pose3D getDeltaPose3DToTag(final AprilTagDetection detection) {
        return Pose3D.from2D(
                getDeltaPoseToTag(detection),
                (float) detection.metadata.distanceUnit.toInches(detection.ftcPose.z) - camPose.translation.z,
                (float) detection.ftcPose.pitch - camPose.rotation.y
        );
    }

    public static Pose2D getRobotPose(final AprilTagDetection detection) {
        return new Pose2D(
                (float) detection.robotPose.getPosition().x,
                (float) detection.robotPose.getPosition().y,
                (float) detection.robotPose.getOrientation().getYaw()
        );
    }

    public static Pose3D getRobotPose3D(final AprilTagDetection detection) {
        return Pose3D.from2D(
                getRobotPose(detection),
                (float) detection.robotPose.getPosition().z,
                (float) detection.robotPose.getOrientation().getPitch()
        );
    }

    public static ArrayList<Point> getTagCorePointsInFrame(final AprilTagDetection detection) {
        ArrayList<Point> corePoints = new ArrayList<>();
        Collections.addAll(corePoints, detection.corners);
        corePoints.add(detection.center);
        return corePoints;
    }

    public static int getId(final AprilTagDetection detection) {
        return detection.id;
    }

    public static float getTagSize(final AprilTagDetection detection) {
        return (float) detection.metadata.tagsize;
    }

    public static String getTagName(final AprilTagDetection detection) {
        return detection.metadata.name;
    }

    public static Pose2D getTagPose(final AprilTagDetection detection) {
        return getRobotPose(detection).add(getDeltaPoseToTag(detection));
    }

    public static TagInfo getTagInfo(final AprilTagDetection detection) {
        return new TagInfo(
                detection.metadata.name,
                detection.id,
                (float) detection.metadata.tagsize,
                getTagCorePointsInFrame(detection),
                getDeltaPoseToTag(detection),
                getTagPose(detection)
        );
    }
}
