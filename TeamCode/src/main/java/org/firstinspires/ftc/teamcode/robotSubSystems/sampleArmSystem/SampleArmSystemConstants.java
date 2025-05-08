package org.firstinspires.ftc.teamcode.robotSubSystems.sampleArmSystem;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robotData.Constants;

@Config // for the dashboard
public class SampleArmSystemConstants {
    public static final float travelAngleRads = 0; // in rads
    public static final float intakeAngleRads = Constants.PI / 4; // 45 deg in rads
    public static final float placeAngleRads = Constants.PI / 2; // 90 deg in rads
    public static final float kp = 0f; // for kp in MotorControlParams // TODO tune
    public static final float ki = 0f; // for ki in MotorControlParams // TODO tune
    public static final float kd = 0f; // for kd in MotorControlParams // TODO tune
    public static final float iZone = 0f; // for iZone in MotorControlParams // TODO tune
    public static final float ks = 0f; // // for ks in MotorControlParams // TODO tune
    public static final float kv = 0f; // for kv in MotorControlParams // TODO tune
    public static final float kg = 0f; // for kg in calc gPower // TODO tune
    public static final float angleTolerance = 0f;
}
