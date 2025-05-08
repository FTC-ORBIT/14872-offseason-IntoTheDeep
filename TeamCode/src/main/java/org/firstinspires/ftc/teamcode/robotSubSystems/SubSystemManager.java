package org.firstinspires.ftc.teamcode.robotSubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitGamepad.ButtonsId;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitLEDS.OrbitLEDBlinkin;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.ScoringAutomator;
import org.firstinspires.ftc.teamcode.autonomous.OrbitAutonomousGeneral;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.roadRunner_1_0.Drawing;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.Assists.DriveByAprilTags.DriveByAprilTags;
import org.firstinspires.ftc.teamcode.OrbitHardware.Sensors.OrbitGyro;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class SubSystemManager {

    public static RobotState lastState = RobotState.TRAVEL;

    public static RobotState wantedState = RobotState.TRAVEL;
    private static boolean blink = false;
    public static Pose2D robotPos;

    private static RobotState getStateFromDriver(Gamepad gamepad) {

        if (GlobalData.firstCycleInTeleop) {
            lastState = GlobalData.lastStateAuto;
        }

        wantedState = gamepad.b ? RobotState.TRAVEL
                : gamepad.a ? RobotState.INTAKE
                : lastState;

        // TODO => chose one option, if the orbitGamePad works use this one:
        wantedState = GlobalData.driverRawButtons.get(ButtonsId.B) ? RobotState.TRAVEL
                : GlobalData.driverRawButtons.get(ButtonsId.A) ? RobotState.INTAKE
                : lastState;



        return wantedState;
    }

    private static RobotState getStateFromRobot(RobotState currentState) {
        if (!GlobalData.robotState.equals(currentState) && !GlobalData.robotState.equals(lastState)){
            return GlobalData.robotState;
        }
        if (currentState.equals(RobotState.INTAKE) & GlobalData.hasGamePiece) {
            return RobotState.TRAVEL;
        }

        return currentState;
    }

    public static RobotState getState(Gamepad gamepad) {
        RobotState currentState = getStateFromDriver(gamepad);
        currentState = getStateFromRobot(currentState);
        GlobalData.robotState = currentState;
        if (lastState != currentState) {
            lastState = currentState;
        }
        return currentState;
    }


    public static void setSubsystemToState(RobotState currentState, boolean resetGyro) {

        robotPos =
                DriveByAprilTags.seeTag() ? Pose2D.fromAprilTags(DriveByAprilTags.getDetection())
                        : GlobalData.inAutonomous ? Pose2D.fromRR(OrbitAutonomousGeneral.roadRunnerDrive.getPose()) // switch to pedro if you use it
                        : DrivetrainOmni.getPose(); // if you don't have deadwheels connected change the DriveTrainOmni.getPose() to Pose2D.zero

        OrbitPoseTracker.setPose(robotPos);

        switch (currentState) {
            case TRAVEL:
                break;
            case INTAKE:
                break;
        }

        ScoringAutomator.update();
        ScoringAutomator.processAssists();


        OrbitLEDBlinkin.operate(currentState, blink);
        if (resetGyro) OrbitGyro.resetGyro();
    }


    public static void printData(Telemetry telemetry, TelemetryPacket packet, FtcDashboard dashboard) {
        if (GlobalData.inCompetition) {
            telemetry.addData("Robot current state ", wantedState);
            telemetry.addData("drive factor", DrivetrainOmni.driveFactor);
            telemetry.addData("gyro angle", OrbitGyro.getAngle());
            telemetry.addLine("robot pos:");
            telemetry.addLine(robotPos.toString());
            telemetry.update();
        } else {
            packet.put("Robot current state ", wantedState);
            packet.put("drive factor", DrivetrainOmni.driveFactor);
            packet.put("gyro angle", OrbitGyro.getAngle());
            packet.addLine("robot pos:");
            packet.addLine(robotPos.toString());

            if (GlobalData.wasInAutonomous) {
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), Pose2D.toRR(robotPos));
            }

            dashboard.sendTelemetryPacket(packet);
        }
    }

    public static void updateGlobalData(Gamepad gamepad2) {
        if (gamepad2.a) GlobalData.allianceColor = !GlobalData.allianceColor;
        if (gamepad2.b) GlobalData.hasGamePiece = !GlobalData.hasGamePiece;
        if (gamepad2.x) {
            GlobalData.allowDriveByAprilTagsAssistAndAutoDrive = !GlobalData.allowDriveByAprilTagsAssistAndAutoDrive;
        }
        if (gamepad2.y) {
            GlobalData.allowDriveByObjectsAssistAndAutoDrive = !GlobalData.allowDriveByObjectsAssistAndAutoDrive;
        }
    }


    public static boolean systemsReady() {
        return false; // TODO => add your systems here
    }
}

//dani yalechan!
// yoel yalechan!
// eitan yalechan!