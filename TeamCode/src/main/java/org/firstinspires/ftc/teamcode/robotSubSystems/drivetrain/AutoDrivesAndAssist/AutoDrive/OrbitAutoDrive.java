package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.AutoDrive;

import org.firstinspires.ftc.teamcode.OrbitUtils.OrbitLogger.CSVLogger;
import org.firstinspires.ftc.teamcode.OrbitUtils.OrbitLogger.CSVTitle;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmniConstants;

public abstract class OrbitAutoDrive {

    public static void moveRobot(final Vector vel, final float rotation) {
        GlobalData.inAutoDrive = !GlobalData.assistActive;



        DrivetrainOmni.drive(vel, rotation);
    }
    public static void moveRobot(final Pose2D velocity) {
        GlobalData.inAutoDrive = !GlobalData.assistActive;


        DrivetrainOmni.drive(velocity.translation, velocity.rotation);
    }
}
