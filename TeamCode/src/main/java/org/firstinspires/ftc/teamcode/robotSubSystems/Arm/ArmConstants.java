package org.firstinspires.ftc.teamcode.robotSubSystems.Arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {
    public static final float travelAngle = -0.71f;
    public static final float highTravelAngle = -0.3f;
    public static final float servoTravelPos = 0.24f;

    public static final float highBasketAngle = 1.6126f; // -12850f;
    public static final float servoHighBasketPos = 0.60f;// 0.38f;

    public static final float lowBasketAngle = 1.6126f; // -12850f;
    public static final float servoLowBasketPos = 0.39f; //0.6f; //0.2f;

    public static final float climbAngle = 1.745f; //-13970f;

    public static final float intakeAngle = 1.745f; //-13970f;
    public static final float servoIntakePos = 1.745f; //-13970f;

    public static final float ascendArmPos = 0.604f; //-13970f;
    public static final float servoAscendPos = 0.75f;

    public static final float intakeAutoPos = -0.455f; //-13970f;
    public static final float servoIntakeAutoPos = 0.72f;

    public static final float subShow = -0.3f; //-13970f;
    public static final float servoSubShow = 0.55f;

    public static final float prepIntakeAutoPos = -0.437f; //-13970f;
    public static final float servoPrepIntakeAutoPos = 0.72f;

    public static final float dragSpikeAutoPos = -0.29f;
    public static final float servoDragSpikeAutoPos = 0.69f;

    public static final float wallIntakePos = -0.71f;
    public static final float wallIntakeEndPos = -0.2f; // -2244f
    public static final float servoWallIntake = 0.305f;
    public static final float servoWallIntakeDeltaArranging = 0f;

    public static final float FrontHighChamberPos = 0.90f; // -8621f;   new
    public static final float servoFrontChamberPos = 0.9f; // 0.2
    public static final float servoFrontChamberPosPlacing = 0.81f; // 0.2

    public static final float backChamberPos = 1.3f; // -13660f;
    public static final float servoBackChamberPos = 0.45f; // 0.2

    public static final float highChamberPosPlacing = 0.8f; // -13660f;
    public static final float servoChamberPosPlacing = 0.36f;

    public static final float afterWallFrontChamberPos = 1.0618f; //-6341    -9401
    public static final float servoAfterWallFrontChamberPos = 0.81f; // 0.12   0.81
    public static final float servoAfterWallFrontChamberPosPlacing = 0.92f; // 0.12     0.92

    public static final float afterWallBackChamberPos = 1.33f; // added 0.04 after 0.02+ // -10700
    public static final float servoAfterWallBackChamberPos = 0.24f;
    public static final float servoAfterWallBackChamberPosPlacing = 0.35f;

    public static final float servoSubIntakePos = 0.83f;
    public static final float servoSubKick = 0.95f;

    public static final float servoIntakePosHorz = 0.77f; // 0.52
    public static final float servoIntakePosVert = 1.0f;

    public static final float KP = 0.00005f;
    public static final float KI = 0f;
    public static final float KD = 0f;
    public static final float KV = 0.1f;
    public static final float KS = 0.02f;
    public static final float KG = 0f;
    public static final float Izone = 0f;

    public static final float armWheelDiameter = 0.075f; // meter
    public static final float gearRatio = 4f; // bigger - power, smaller - vel
    public static final float TicksInRad = -5303f;
    public static final float horizontalTicks = -3768f;
    public static final float overrideFactor = 0f;


}
