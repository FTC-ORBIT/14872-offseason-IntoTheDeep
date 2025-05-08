package org.firstinspires.ftc.teamcode.testsOpModes;

import org.firstinspires.ftc.teamcode.OrbitHardware.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.autonomous.OrbitAutonomousGeneral;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.AutoDrive.AutoDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainTank.DriveTrainTank;

public class AutoDriveTests extends OrbitAutonomousGeneral {
    public static final Class<?> DRIVE_CLASS = DrivetrainOmni.class;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        if (DRIVE_CLASS == DrivetrainOmni.class) {
            DrivetrainOmni.init(hardwareMap);
        } else if (DRIVE_CLASS == DriveTrainTank.class) {
            DriveTrainTank.init(hardwareMap);
        } else {
            throw new RuntimeException("Invalid drive class");
        }
        OrbitGyro.init(hardwareMap);
        GlobalData.inAutonomous = false;
        GlobalData.wasInAutonomous = false;
        OrbitPoseTracker.setPose(Pose2D.zero());

        waitForStart();

        int state = 0;
        boolean lastRightBumper = false;
        boolean lastLeftBumper = false;
        AutoDrive autoDrive = new AutoDrive(1, Angle.degToRad(3), 0.2f, 1);

        while (!isStopRequested()) {
            if (gamepad1.right_bumper && !lastRightBumper) {
                state++;
            } else if (gamepad1.left_bumper && !lastLeftBumper) {
                state--;
            } else if (gamepad1.a) {
                state = 0;
            }
            switch (state) {
                default:
                case 0:
                    if (DRIVE_CLASS == DrivetrainOmni.class) {
                        DrivetrainOmni.stop();
                    } else {
                        DriveTrainTank.stop();
                    }
                    break;
                case 1:
                    autoDrive.driveTo(Pose2D.zero());
                    break;
                case 2:
                    autoDrive.driveTo(new Pose2D(0, 0, Angle.halfPI));
                    break;
                case 3:
                    autoDrive.rotateTo(Angle.pi);
                    break;
                case 4:
                    autoDrive.driveTo(new Pose2D(30, 30, 0));
                    break;
                case 5:
                    autoDrive.driveTo(new Pose2D(30, 30, Angle.halfPI));
                    break;
                case 6:
                    autoDrive.driveToLerp(new Pose2D(30, 30, Angle.halfPI));
                    break;
                case 7:
                    autoDrive.driveToLerp(new Pose2D(30, 30, 0));
                    break;
            }
            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;
            telemetry.addData("State", state);
            telemetry.update();
        }
    }

}

