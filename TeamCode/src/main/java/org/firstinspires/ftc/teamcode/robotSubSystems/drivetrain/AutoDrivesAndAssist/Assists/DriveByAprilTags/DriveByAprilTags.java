package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.Assists.DriveByAprilTags;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.Assists.OrbitAssists;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.AutoDrive.AutoDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitCams.OrbitWebcam.TempVision.aprilTags.OrbitAprilTag;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class DriveByAprilTags extends OrbitAssists {

    public final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private static AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    public static AprilTagDetection tagDetected = null; // Used to hold the data for a detected AprilTag
    public final float FINAL_DESIRED_DISTANCE = 16;
    public float DESIRED_DISTANCE = FINAL_DESIRED_DISTANCE;
    private final AutoDrive tagsAutoDrive = new AutoDrive(1, Angle.degToRad(3), 10f,40f);


    @Override
    public boolean shouldActivateAssist() {
        return seeTarget() && GlobalData.allowDriveByAprilTagsAssistAndAutoDrive;
    }

    public void initProcessor() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .setLensIntrinsics(458.066, 457.626, 337.176, 251.805)
                .build();


        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) setManualExposure(8, 250);  // Use low exposure time to reduce motion blur
    }


    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            }

        }

        // Set camera controls unless we are stopping.

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    public void update() {

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    tagDetected = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    tagDetected = detection;
                    targetFound = false;
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                targetFound = false;
            }
        }
    }


    public void autoDriveToAprilTagAutonomous(MecanumDrive autoDrive) {
        final Pose2D tagPose = OrbitAprilTag.getTagPose(tagDetected);

        final Pose2D targetPose = new Pose2D(
                Vector.fromAngleAndRadius(tagPose.translation.getAngle(),
                        tagPose.translation.norm() - FINAL_DESIRED_DISTANCE)
                , tagPose.rotation
        );

        autoDrive.setPowerForAutoDrives(tagsAutoDrive.calcVel(targetPose));
    }

    public boolean seeTarget() {
        return targetFound;
    }

    public static boolean seeTag() {
        return !aprilTag.getDetections().isEmpty();
    }

    public static AprilTagDetection getDetection() {
        return tagDetected;
    }

    public void assistByTarget() {

        final Pose2D tagPose = OrbitAprilTag.getTagPose(tagDetected);

        final Pose2D targetPose = new Pose2D(
                Vector.fromAngleAndRadius(tagPose.translation.getAngle(),
                        (tagPose.translation.norm() - FINAL_DESIRED_DISTANCE)
                ), tagPose.rotation
        ); // the distance right now is for the 2 axis

        GlobalData.assistActive = true;
        moveRobot(tagsAutoDrive.calcVel(targetPose).add(DrivetrainOmni.getVelFromDriver()));
    }

    public void turnToTagAutoDriveAutonomous(MecanumDrive drive) {
        double headingError = tagDetected.ftcPose.bearing;
        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        drive.setPowerForAutoDrives(Vector.zero(), turn);
    }

    public void turnToTagAssist() {
        final float angularVel = tagsAutoDrive.calcAngularVel(OrbitAprilTag.getTagPose(tagDetected).rotation - OrbitPoseTracker.getHeading()) + DrivetrainOmni.getOmega();
        GlobalData.assistActive = true;
        moveRobot(DrivetrainOmni.getPositionVelFromDriver(), angularVel);
    }

    public void autoDriveToTag() {
       final Pose2D tagPose = OrbitAprilTag.getTagPose(tagDetected);
         final Pose2D targetPose = new Pose2D(
                Vector.fromAngleAndRadius(tagPose.translation.getAngle(),
                          (tagPose.translation.norm() - FINAL_DESIRED_DISTANCE)
                ), tagPose.rotation
         );
        moveRobot(tagsAutoDrive.calcVel(targetPose));
    }

    public void turnToTagAutoDrive() {
        final float angularVel = tagsAutoDrive.calcAngularVel(OrbitAprilTag.getTagPose(tagDetected).rotation);
        moveRobot(Vector.zero(), angularVel);
    }

    public void setDesiredDistance(final float distance) {
        DESIRED_DISTANCE = distance;
    }

    public AprilTagProcessor getProcessor() {
        return aprilTag;
    }
}

