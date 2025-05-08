package org.firstinspires.ftc.teamcode.autonomous.RR.enable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.OrbitAutonomousGeneral;
import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;
@Config
@Autonomous (group = "tests")
public class OrbitRRTuneTest extends OrbitAutonomousGeneral {
    public static boolean isFront = true;
    public static boolean withRotation = true;
    public static boolean isLeft = true;
    @Override
    public void runOpMode() throws InterruptedException {
        roadRunnerDrive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
        initRobot();
        waitForStart();
        final float dis = 72;
        TrajectoryActionBuilder driveFront = roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
                .lineToX(dis)
                .lineToX(0);

        TrajectoryActionBuilder driveLeft = roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
                .lineToY(dis)
                .lineToY(0);

        TrajectoryActionBuilder driveRight = roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
                .lineToY(-dis)
                .lineToY(0);

        TrajectoryActionBuilder driveLeftWithTurn = roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
                .lineToYLinearHeading(dis,Math.toRadians(180))
                .lineToYLinearHeading(0,Math.toRadians(0));

        TrajectoryActionBuilder driveRightWithTurn = roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
                .lineToYLinearHeading(-dis,Math.toRadians(180))
                .lineToYLinearHeading(0,Math.toRadians(0));

        TrajectoryActionBuilder driveFrontWithTurn = roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
                .lineToXLinearHeading(dis,Math.toRadians(180))
                .lineToXLinearHeading(0,Math.toRadians(0));

        if (isFront && withRotation){
            Actions.runBlocking(driveFrontWithTurn.build());
        } else if (isFront) {
            Actions.runBlocking(driveFront.build());
        }else if (isLeft && withRotation){
            Actions.runBlocking(driveLeftWithTurn.build());
        } else if (isLeft) {
            Actions.runBlocking(driveLeft.build());
        } else if (withRotation) {
            Actions.runBlocking(driveRightWithTurn.build());
        } else {
            Actions.runBlocking(driveRight.build());
        }
    }

}

