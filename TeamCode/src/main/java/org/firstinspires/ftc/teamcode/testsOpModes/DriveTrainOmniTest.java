package org.firstinspires.ftc.teamcode.testsOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;

import org.firstinspires.ftc.teamcode.OrbitHardware.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;

@Config
@TeleOp(name = "test")
public class DriveTrainOmniTest extends LinearOpMode {


    private float addKs(float value, final float unDeadBandVal, final float ks) {
        if (Math.abs(value) > unDeadBandVal) {
            value += ks * Math.signum(value);
        }
        return value;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        GlobalData.inAutonomous = false;

        OrbitPoseTracker.setPose(Pose2D.zero());
        ElapsedTime robotTime = new ElapsedTime();
        robotTime.reset();
        DrivetrainOmni.init(hardwareMap);
        OrbitGyro.init(hardwareMap);


        OrbitGyro.resetGyroStartTeleop((float) OrbitPoseTracker.getHeading());
        telemetry.addData("gyro", OrbitGyro.getAngle());
        telemetry.addData("lastAngle", OrbitGyro.lastAngle);
        telemetry.update();

        GlobalData.currentTime = 0;
        GlobalData.lastTime = 0;
        GlobalData.deltaTime = 0;
        GlobalData.robotState = RobotState.TRAVEL;
        GlobalData.hasGamePiece = false;

        float factorizedY;
        float factorizedX;
        boolean usingDashboardForDriving = false;
        Vector leftStick;
        float omega;
        waitForStart();

        int state = 0;
        boolean lastRightBumper = false;
        boolean lastLeftBumper = false;
        String test = "none";
        while (!isStopRequested()) {
            if (gamepad1.right_bumper && !lastRightBumper) {
                state++;
            } else if (gamepad1.left_bumper && !lastLeftBumper) {
                state--;
            } else if (gamepad1.a) {
                state = 0;
            }
            switch (state) {
                case 0:
                    DrivetrainOmni.stop();
                    test = "none";
                    break;
                case 1:
                    DrivetrainOmni.testMotors(gamepad1, telemetry);
                    test = "test Motors";
                    break;
                case 2:
                    DrivetrainOmni.testEncoder(telemetry);
                    test = "test encoders";
                    break;
                case 3:
                    DrivetrainOmni.testDeadWheels(telemetry);
                    test = "test dead wheels";
                    break;
                case 4:
                    factorizedY = gamepad1.left_stick_y * (float) Math.sqrt(Math.abs(gamepad1.left_stick_y));
                    factorizedX = gamepad1.left_stick_x * (float) Math.sqrt(Math.abs(gamepad1.left_stick_x));

                    factorizedX = addKs(factorizedX, 0.01f, 0.1f); // TODO tune!
                    factorizedY = addKs(factorizedY, 0.01f, 0.1f); // TODO tune!
                    usingDashboardForDriving = dashboard != null & GlobalData.usingDashBoardGampad;
                    leftStick = new Vector(usingDashboardForDriving ? -factorizedY : factorizedX, usingDashboardForDriving ? factorizedX : -factorizedY);
                    omega = gamepad1.right_trigger - gamepad1.left_trigger;
                    DrivetrainOmni.drive(leftStick, omega);
                    test = "drive without field centric";
                    break;
                case 5:
                    factorizedY = gamepad1.left_stick_y * (float) Math.sqrt(Math.abs(gamepad1.left_stick_y));
                    factorizedX = gamepad1.left_stick_x * (float) Math.sqrt(Math.abs(gamepad1.left_stick_x));

                    factorizedX = addKs(factorizedX, 0.01f, 0.1f); // TODO tune!
                    factorizedY = addKs(factorizedY, 0.01f, 0.1f); // TODO tune!
                    usingDashboardForDriving = dashboard != null & GlobalData.usingDashBoardGampad;
                    leftStick = new Vector(usingDashboardForDriving ? -factorizedY : factorizedX, usingDashboardForDriving ? factorizedX : -factorizedY);
                    omega = gamepad1.right_trigger - gamepad1.left_trigger;
                    DrivetrainOmni.operate(leftStick, omega);
                    test = "drive with field centric";
                    break;
            }
            if (gamepad1.b) {
                OrbitGyro.resetGyro();
            }
            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;
            telemetry.addData("state =", state);
            telemetry.addData("test =", test);
            telemetry.update();
        }
    }
}
//dani yalechan!
// yoel yalechan!