package org.firstinspires.ftc.teamcode.autonomous.AutoDrives.Enable;

import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.OrbitUtils.OrbitLogger.CSVLogger;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.autonomous.AutoDrives.AutoTrajectory;
import org.firstinspires.ftc.teamcode.autonomous.AutoDrives.AutoTrajectoryPoint;
import org.firstinspires.ftc.teamcode.autonomous.OrbitAutonomousGeneral;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;

@Autonomous(name = "sampleAutoOpMode", group = "Enable", preselectTeleOp = "main")
public class SampleAutoOpMode extends OrbitAutonomousGeneral {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {
        // defining the trajectory points:
        final AutoTrajectoryPoint[] autoTrajectoryPoints = {
                new AutoTrajectoryPoint(new Pose2D(0, 0, 0), new Pose2D(0, 0, 0), RobotState.TRAVEL, false),
                new AutoTrajectoryPoint(new Pose2D(1, 1, 0), new Pose2D(1, 1, 0), RobotState.INTAKE, true),
                new AutoTrajectoryPoint(new Pose2D(2, 2, 0), new Pose2D(2, 2, 0), RobotState.TRAVEL, false),
                new AutoTrajectoryPoint(new Pose2D(3, 3, 0), new Pose2D(3, 3, 0), RobotState.INTAKE, true)
        };

        // creating the trajectory:
        final AutoTrajectory trajectory = new AutoTrajectory(AutoTrajectory.TrajectoryType.NORMAL, autoTrajectoryPoints);

        // it is important to init the robot before starting the trajectory:
        initRobot();
        CSVLogger.init();

        waitForStart();

        while (!isStopRequested()) {
            //running the trajectory:
            trajectory.runTrajectory();

            // update logs:
            telemetry.addLine("robot pose" + OrbitPoseTracker.getRobotOrbitPose2D());
            telemetry.addLine(trajectory.getCurrentTrajectoryPoint().toString());
            telemetry.addData("Arrived", trajectory.arrived());
            telemetry.addData("points left", autoTrajectoryPoints.length - trajectory.getAutoTrajectoryPointIndex());
            telemetry.update();

            packet.addLine("robot pose" + OrbitPoseTracker.getRobotOrbitPose2D());
            packet.addLine(trajectory.getCurrentTrajectoryPoint().toString());
            packet.put("Arrived", trajectory.arrived());
            packet.put("points left", autoTrajectoryPoints.length - trajectory.getAutoTrajectoryPointIndex());
            dashboard.sendTelemetryPacket(packet);

            CSVLogger.update();

        }
        CSVLogger.save();
    }
}
