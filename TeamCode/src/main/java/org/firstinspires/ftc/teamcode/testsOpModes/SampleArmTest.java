package org.firstinspires.ftc.teamcode.testsOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitHardware.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.sampleArmSystem.SampleArmSystem;
import org.firstinspires.ftc.teamcode.robotSubSystems.sampleArmSystem.SampleArmSystemStates;

public class SampleArmTest extends LinearOpMode {

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
        SampleArmSystem.init(hardwareMap, "leftMotor","rightMotor");
        telemetry.addData("gyro", OrbitGyro.getAngle());
        telemetry.addData("lastAngle", OrbitGyro.lastAngle);
        telemetry.update();

        GlobalData.currentTime = 0;
        GlobalData.lastTime = 0;
        GlobalData.deltaTime = 0;
        GlobalData.robotState = RobotState.TRAVEL;
        GlobalData.hasGamePiece = false;


        SampleArmSystemStates lastState = SampleArmSystemStates.TRAVEL;

        waitForStart();

        int state = 0;
        boolean lastRightBumper = false;
        boolean lastLeftBumper = false;
        String test = "none";

        while (!isStopRequested()) {
            SampleArmSystemStates systemState = gamepad1.b ? SampleArmSystemStates.TRAVEL : gamepad1.x ? SampleArmSystemStates.INTAKE : gamepad1.y ? SampleArmSystemStates.PLACE : lastState ;

            if (gamepad1.right_bumper && !lastRightBumper) {
                state++;
            } else if (gamepad1.left_bumper && !lastLeftBumper) {
                state--;
            } else if (gamepad1.a) {
                state = 0;
            }
            switch (state) {
                case 0:
                    SampleArmSystem.setSystemPower(0);
                    test = "none";
                    break;
                case 1:
                    SampleArmSystem.setSystemPower(0.3f);
                    test = "test direction + getting power";
                    break;
                case 2:
                    SampleArmSystem.operate(systemState);
                    test = "test states";
                    break;
                case 3:
                    SampleArmSystem.tune(telemetry);
                    test = "tuning our targets";
                    break;

            }
            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;
            telemetry.addData("state =", state);
            telemetry.addData("test =", test);
            telemetry.update();
        }
    }
}
