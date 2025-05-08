package org.firstinspires.ftc.teamcode.roadRunner_1_0.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.OrbitHardware.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.autonomous.OrbitAutonomousGeneral;
import org.firstinspires.ftc.teamcode.roadRunner_1_0.Drawing;
import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner_1_0.TankDrive;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

@TeleOp(group = "test")
public class LocalizationTest extends OrbitAutonomousGeneral {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    private float addKs(float value, final float unDeadBandVal, final float ks) {
        if (Math.abs(value) > unDeadBandVal) {
            value += ks * Math.signum(value);
        }
        return value;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, Pose2D.toRR(switchStartPos(gamepad1)));
            initRobot();
            waitForStart();

            while (opModeIsActive()) {
                float factorizedY = gamepad1.left_stick_y * (float) Math.sqrt(Math.abs(gamepad1.left_stick_y));
                float factorizedX = gamepad1.left_stick_x * (float) Math.sqrt(Math.abs(gamepad1.left_stick_x));

                factorizedX = addKs(factorizedX, 0.01f, 0.1f); // TODO tune!
                factorizedY = addKs(factorizedY, 0.01f, 0.1f); // TODO tune!
                boolean usingDashboardForDriving = dashboard != null & GlobalData.usingDashBoardGampad;
                Vector leftStick = new Vector(usingDashboardForDriving ? factorizedY : factorizedX, usingDashboardForDriving ? factorizedX : factorizedY);
                float omega = gamepad1.right_trigger - gamepad1.left_trigger;

                drive.setDrivePowersOrbitWay(leftStick,omega);


                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
               dashboard.sendTelemetryPacket(packet);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
