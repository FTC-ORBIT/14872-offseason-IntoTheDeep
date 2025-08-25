package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitGamepad.OrbitGamepad;
import org.firstinspires.ftc.teamcode.OrbitHardware.HardwareColors;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitLEDS.OrbitLEDBlinkin;
import org.firstinspires.ftc.teamcode.OrbitUtils.OrbitLogger.CSVLogger;
import org.firstinspires.ftc.teamcode.OrbitUtils.OrbitCanvas;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.autonomous.OrbitAutonomousGeneral;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.OrbitHardware.Sensors.OrbitGyro;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.Pinch.Pinch;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
import org.firstinspires.ftc.teamcode.robotSubSystems.Telescope.Telescope;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;

@Config
@TeleOp(name = "main")
public class Robot extends LinearOpMode {


    public static TelemetryPacket packet;
    private final OrbitGamepad orbitGamepad1 = new OrbitGamepad();

    private final OrbitCanvas canvas = new OrbitCanvas("Canvas", 800, 800);

    private float addKs(float value, final float unDeadBandVal, final float ks) {
        if (Math.abs(value) > unDeadBandVal) {
            value += ks * Math.signum(value);
        }
        return value;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        if (!GlobalData.wasInAutonomous) {
            OrbitPoseTracker.setPose(Pose2D.zero());
        } else {
            OrbitPoseTracker.setPose(Pose2D.fromRR(OrbitAutonomousGeneral.roadRunnerDrive.getPose())); // switch to pedro if you use it
        }

        GlobalData.inAutonomous = false;

        final FtcDashboard dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();


        ElapsedTime robotTime = new ElapsedTime();
        robotTime.reset();
        DrivetrainOmni.init(hardwareMap);
        Arm.init(hardwareMap, "armMotor", "armMotor2","armServo");
        Telescope.init(hardwareMap,"telMotor");
        Pinch.init(hardwareMap, "pinchServo");
        Intake.init(hardwareMap, "cargoServo1", "cargoServo2");
        ScoringAutomator.initAssists(hardwareMap,"webcam1","webcam2");
        OrbitGyro.init(hardwareMap);
        OrbitLEDBlinkin.init(hardwareMap);
        CSVLogger.init();
        OrbitGyro.resetGyroStartTeleop((float) Math.toDegrees(OrbitPoseTracker.getHeading()));

        telemetry.addData("gyro", OrbitGyro.getAngle());
        telemetry.addData("lastAngle", OrbitGyro.lastAngle);
        telemetry.update();

        GlobalData.currentTime = 0;
        GlobalData.lastTime = 0;
        GlobalData.deltaTime = 0;
        GlobalData.robotState = RobotState.TRAVEL;
        GlobalData.firstCycleInTeleop = true;

        waitForStart();


        while (!isStopRequested()) {
            GlobalData.voltage = (float) hardwareMap.voltageSensor.iterator().next().getVoltage();

            // custom gamePad use :
            orbitGamepad1.copy(gamepad1);

            orbitGamepad1.updateButtons();

            orbitGamepad1.rumble();

            orbitGamepad1.setLedSColor(HardwareColors.ORBIT);

            GlobalData.currentTime = (float) robotTime.seconds();

            //Drive:
            final Vector leftStick = orbitGamepad1.leftJoyStick();
            float factorizedY = leftStick.y * (float) Math.sqrt(Math.abs(leftStick.y));
            float factorizedX = leftStick.x * (float) Math.sqrt(Math.abs(leftStick.x));

            factorizedX = addKs(factorizedX, 0.01f, 0.1f); // TODO tune!
            factorizedY = addKs(factorizedY, 0.01f, 0.1f); // TODO tune!
            final boolean usingDashboardForDriving = dashboard != null & GlobalData.usingDashBoardGampad;
            final Vector driverVel = new Vector(usingDashboardForDriving ? -factorizedY : factorizedX, usingDashboardForDriving ? factorizedX : -factorizedY);
            final float omega = gamepad1.right_trigger - gamepad1.left_trigger;
            DrivetrainOmni.operate(driverVel.switchBetweenAxis(),omega);

            GlobalData.rightStick = orbitGamepad1.rightJoyStick();
//          DriveTrainTank.operate(gamepad1.left_stick_y,gamepad1.right_trigger,gamepad1.left_trigger,telemetry,gamepad1);


            //Systems:
            final RobotState currentState = SubSystemManager.getState(gamepad1);

            SubSystemManager.setSubsystemToState(currentState, gamepad1.dpad_down);

            SubSystemManager.updateGlobalData(gamepad2);

            SubSystemManager.printData(telemetry, packet, dashboard);



            // otherLogic:
            CSVLogger.update();
            canvas.drawRobot(HardwareColors.ORBIT);

            if (dashboard != null) {
                canvas.sendToDashboard(dashboard, packet);
            }
            canvas.printToDriverHub(telemetry);

            GlobalData.firstCycleInTeleop = false;


            GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
            GlobalData.lastTime = GlobalData.currentTime;
        }
        CSVLogger.save();
    }


}
//dani yalechan!
// yoel yalechan!