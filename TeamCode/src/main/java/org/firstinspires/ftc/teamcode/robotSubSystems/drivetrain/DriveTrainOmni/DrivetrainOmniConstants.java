package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni;

import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;

public class DrivetrainOmniConstants {
    public static final float power = 1f;
    public static final float slowPower = 0.6f;
    public static final float superSlowPower = 0.4f;
    public static final float minPower = 0.2f;
    public static final float omega = 1f;
    public static final float slowOmega = 0.6f;
    public static final float superSlowOmega = 0.4f;
    public static final float minOmega = 0.2f;

    public static final float minVoltageForNormalDrive = 12f;
    public static final float minVoltageForSlowDrive = 10f;
    public static final float minVoltageForSuperSlowDrive = 8f;

    public static final float inPerTick = (float) MecanumDrive.PARAMS.inPerTick;

    public static final float maxVel = 55f; // inches per second // TODO - tune!
    public static final float maxAccel = 40f; // inches per second ^ 2 // TODO - tune!
    public static final float maxJerk = 120f; // inches per second ^ 3 // TODO - tune!

    public static final float maxAngularVel = Angle.pi; // TODO - tune!
    public static final float maxAngularAccel = 2f; // TODO - tune!
    public static final float maxAngularJerk = 6f; // TODO - tune!

    public static final float minAccel = 0.5f; //inches per second ^ 2 // TODO - tune!
    public static final float minAngularAccel = 0.2f ; // rads per second ^ 2 // TODO - tune!


    public static final float ks = 0.1f; // TODO - tune!
}
