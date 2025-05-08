package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.Assists.DriveByObjects;
import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose3D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.Assists.OrbitAssists;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.AutoDrive.AutoDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;

public class DriveByObjects extends OrbitAssists {
    private static final ColorObjectTracker colorObjectTracker = new ColorObjectTracker();
    private static final float DESIRED_DISTANCE = 5; // in inches
    private final AutoDrive objectAutoDrive = new AutoDrive(1, Angle.degToRad(3), 10,  40);
    private final Pose3D camPose = new Pose3D(0,0,0,0,0,0); // TODO => TUNE!

    @Override
    public boolean shouldActivateAssist() {
        return seeTarget() && GlobalData.robotState.equals(RobotState.INTAKE) && !GlobalData.hasGamePiece && GlobalData.allowDriveByObjectsAssistAndAutoDrive;
    }

    public void initProcessor() {

    }


    public void autoDriveByTarget(final MecanumDrive autoDrive) {
        final Pose2D cameraPose = camPose.to2D();
        final float distance =  colorObjectTracker.getDistance() - DESIRED_DISTANCE;
        final Pose2D objectPose = new Pose2D(
                Vector.fromAngleAndRadius(colorObjectTracker.calcDirection(),distance),
                Angle.wrapPlusMinusPI(colorObjectTracker.getYaw())
        ).add(cameraPose);

        autoDrive.setPowerForAutoDrives(objectAutoDrive.calcVel(objectPose));


    }

    public void assistByTarget() {
        final Pose2D cameraPose = camPose.to2D();
        final float distance =  colorObjectTracker.getDistance() - DESIRED_DISTANCE;
        final Pose2D objectPose = new Pose2D(
                Vector.fromAngleAndRadius(colorObjectTracker.calcDirection(),distance),
                Angle.wrapPlusMinusPI(colorObjectTracker.getYaw())
        ).add(cameraPose);

        moveRobot(objectAutoDrive.calcVel(objectPose).add(DrivetrainOmni.getVelFromDriver()));
    }

    @Override
    public void update() {
        return;
    }


    public  void turnToTargetAutoDrive(MecanumDrive autoDrive) {
       final float targetHeading = Angle.wrapPlusMinusPI(colorObjectTracker.getYaw() + camPose.rotation.z);

        autoDrive.setPowerForAutoDrives(Vector.zero(),objectAutoDrive.calcAngularVel(targetHeading));

    }

    public void turnToTargetAssist() {
       final float targetHeading = Angle.wrapPlusMinusPI(colorObjectTracker.getYaw() + camPose.rotation.z);
        moveRobot(DrivetrainOmni.getPositionVelFromDriver(), objectAutoDrive.calcAngularVel(targetHeading - OrbitPoseTracker.getHeading()));
    }

    public boolean seeTarget() {
        return colorObjectTracker.isObjectFound();
    }

    public ColorObjectTracker getProcessor() {
        return colorObjectTracker;
    }

}
