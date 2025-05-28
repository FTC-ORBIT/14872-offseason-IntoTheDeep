package org.firstinspires.ftc.teamcode.robotSubSystems.Arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.MotorControlMode;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.MotorControlParams;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.OrbitMotor;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.PositionUnits;
import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;

public class Arm {
    public static OrbitMotor armMotor;
    public static OrbitMotor armMotor2;
    public static Servo armServo;
    public static float currentAngle;
    public static float wantedAngleRads;
    public static float servoPos;
    public static ArmStates currentState;
    public static ArmStates lastState;
    public static float armDegrees;
    public static MotorControlParams armControlParams = new MotorControlParams(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD, ArmConstants.Izone, ArmConstants.KS, ArmConstants.KV);

    public static void init(HardwareMap hardwareMap, String name, String name2, String name3) {
        armMotor = new OrbitMotor(hardwareMap, name, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, armControlParams, ArmConstants.gearRatio, ArmConstants.armWheelDiameter, PositionUnits.RADS);
        armMotor2 = new OrbitMotor(hardwareMap, name2, DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, armControlParams, ArmConstants.gearRatio, ArmConstants.armWheelDiameter, PositionUnits.RADS);
        armServo = hardwareMap.get(Servo.class, name3);
    }

    public static void operate(ArmStates state) {
        switch (state) {
            case TRAVEL:
                wantedAngleRads = ArmConstants.travelAngle;
                servoPos = ArmConstants.servoTravelPos;
                break;

            case HIGH_TRAVEL:
                wantedAngleRads = ArmConstants.highTravelAngle;
                servoPos = ArmConstants.servoHighTravelPos;
                break;

            case INTAKE:
                wantedAngleRads = ArmConstants.intakeAngle;
                servoPos = ArmConstants.servoIntakePos;
                break;

            case HIGH_CHAMBER:
                wantedAngleRads = ArmConstants.highChamberAngle;
                servoPos = ArmConstants.servoHighChamberPos;
                break;

            case LOW_BASKET:
                wantedAngleRads = ArmConstants.lowBasketAngle;
                servoPos = ArmConstants.servoLowBasketPos;
                break;

            case HIGH_BASKET:
                wantedAngleRads = ArmConstants.highBasketAngle;
                servoPos = ArmConstants.servoHighBasketPos;
                break;

            case CLIMB:
                wantedAngleRads = ArmConstants.climbAngle;
                servoPos = ArmConstants.servoTravelPos;
                break;

            case LOW_CHAMBER:
                wantedAngleRads = ArmConstants.lowChamberAngle;
                servoPos = ArmConstants.servoLowChamberPos;
                break;


        }
        final float gForce = ArmConstants.KG * MathFuncs.cos(getAngle());

        armMotor.setPower(MotorControlMode.MOTION_MAGIC_POSITION, wantedAngleRads, gForce);
        armMotor2.slave(armMotor);
        armMotor.setPeak(1f);
        armMotor2.setPeak(1f);




    }

    public static float getAngle() {
        return (armMotor.getCurrentPosition(PositionUnits.RADS) + armMotor2.getCurrentPosition(PositionUnits.RADS)) / 2;

    }
}
