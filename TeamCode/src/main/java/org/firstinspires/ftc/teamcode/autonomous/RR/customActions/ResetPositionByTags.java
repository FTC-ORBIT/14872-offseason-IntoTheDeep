package org.firstinspires.ftc.teamcode.autonomous.RR.customActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.GlobalPos;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class ResetPositionByTags extends GlobalPos {
    private final AprilTagProcessor aprilTagProcessor;
    private final MecanumDrive actionDrive;
    private boolean hadReset = false;
    private final VisionPortal portal;


    public ResetPositionByTags(final WebcamName name, final MecanumDrive actionDrive){
        this.actionDrive = actionDrive;
        aprilTagProcessor = new AprilTagProcessor.Builder()
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
        aprilTagProcessor.setDecimation(2);

         portal = new VisionPortal.Builder()
                .setCamera(name)
                .addProcessor(aprilTagProcessor)
                .build();
    }




    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!hadReset) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            if (currentDetections.get(0).metadata != null) {
               final Pose2d robotPosRRFromTag =  OrbitPoseTracker.getRobotPosRRFromTag(currentDetections.get(0));
                actionDrive.pose.copy(robotPosRRFromTag.position, robotPosRRFromTag.heading);
                hadReset = true;
            }
        }else {
            hadReset = false;
        }


        telemetryPacket.put("Had Reset", hadReset);
        if (hadReset){
            portal.close();
            return false;
        }
        return true;
    }
}
