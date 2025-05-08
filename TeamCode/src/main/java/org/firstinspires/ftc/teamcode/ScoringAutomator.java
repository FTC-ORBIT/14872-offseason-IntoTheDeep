package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitGamepad.ButtonsId;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.Assists.DriveByAprilTags.DriveByAprilTags;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.Assists.DriveByObjects.DriveByObjects;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.Assists.OrbitAssists;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

public class ScoringAutomator {
    private static final DriveByAprilTags driveByAprilTags = new DriveByAprilTags();
    private static final DriveByObjects driveByObjects = new DriveByObjects();
    private static final List<OrbitAssists> assists = List.of(driveByAprilTags, driveByObjects);
    public static CameraName tagsCam = null;
    public static CameraName intakeCam = null;



    public static void initAssists(HardwareMap hardwareMap, String webcamOne, String webcamTwo){

        tagsCam = hardwareMap.get(WebcamName.class,webcamOne);
        intakeCam = hardwareMap.get(WebcamName.class,webcamTwo);
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(tagsCam, intakeCam);

        initProcessor();

        if (OrbitAssists.USE_WEBCAM) {
            OrbitAssists.visionPortal = new VisionPortal.Builder()
                    .setCamera(switchableCamera)
                    .addProcessor(driveByAprilTags.getProcessor())
                    .addProcessor(driveByObjects.getProcessor())
                    .build();
        }else {
            OrbitAssists.visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(driveByObjects.getProcessor())
                    .addProcessor(driveByAprilTags.getProcessor())
                    .build();
        }
    }
    public static void initProcessor() {
        for (OrbitAssists assist : assists) {
            assist.initProcessor();
        }
    }

    public static void processAssists() {
        for (OrbitAssists assist : assists) {
            if (assist.shouldActivateAssist()) {
                GlobalData.assistActive = true;
                assist.assistByTarget();
            }else {
                GlobalData.assistActive = false;
            }
        }
    }

    public static void update() {
        for (OrbitAssists assist : assists) {
            assist.update();
        }
    }

}
