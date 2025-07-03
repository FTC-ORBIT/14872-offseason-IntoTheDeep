package org.firstinspires.ftc.teamcode.robotSubSystems.Arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.MotorControlMode;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.MotorControlParams;

import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.PositionUnits;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.OrbitMotor;
import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.OrbitUtils.MotionMagic;
import org.firstinspires.ftc.teamcode.OrbitUtils.PID;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class Arm {
    //    public static OrbitMotor armMotor;
    static DcMotor motor;
    static DcMotor motor2;
    //    public static OrbitMotor armMotor2;
    public static Servo armServo;
    public static float currentAngle;
    public static float wantedAngleRads;
    public static float currentPos = 0;
    public static float servoPos;
    public static ArmStates currentState;
    public static ArmStates lastState;
    public static float armDegrees;
    public static MotorControlParams armControlParams = new MotorControlParams(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD, ArmConstants.Izone, ArmConstants.KS, ArmConstants.KV);

    public static PID pid = new PID(armControlParams);

    public static void init(HardwareMap hardwareMap, String name, String name2, String name3) {
//        armMotor = new OrbitMotor(hardwareMap, name, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, armControlParams, ArmConstants.gearRatio, ArmConstants.armWheelDiameter, PositionUnits.RADS);
//        armMotor2 = new OrbitMotor(hardwareMap, name2, DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, armControlParams, ArmConstants.gearRatio, ArmConstants.armWheelDiameter, PositionUnits.RADS);
        armServo = hardwareMap.get(Servo.class, name3);
//
        motor = hardwareMap.get(DcMotor.class, name);
        motor2 = hardwareMap.get(DcMotor.class, name2);

//        armMotor.setPeak(0.3f);
//        armMotor2.setPeak(0.3f);
    }

    public static void operate(ArmStates state) {
        switch (state) {
            case TRAVEL:
                wantedAngleRads = ArmConstants.travelAngle;
                servoPos = ArmConstants.servoTravelPos;
                break;

            case HIGH_TRAVEL:
                wantedAngleRads = ArmConstants.highTravelAngle;
                servoPos = ArmConstants.servoTravelPos;
                break;

            case INTAKE:
                wantedAngleRads = ArmConstants.intakeAngle;
                servoPos = ArmConstants.servoIntakePos;
                break;

            case HIGH_CHAMBER:
                wantedAngleRads = ArmConstants.FrontHighChamberPos;
                servoPos = ArmConstants.servoFrontChamberPos;
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
                wantedAngleRads = ArmConstants.backChamberPos;
                servoPos = ArmConstants.servoBackChamberPos;
                break;


        }
//        final float gForce = ArmConstants.KG * MathFuncs.cos(getAngle());

//        armMotor.setPower(MotorControlMode.MOTION_MAGIC_POSITION, wantedAngleRads, gForce);
//        armMotor.setPower(-0.4f);
//        armMotor2.setPower(-0.4f);



        currentPos = (float) (motor.getCurrentPosition() + motor2.getCurrentPosition()) / 2 ;
        pid.setWanted(wantedAngleRads * ArmConstants.TicksInRad);

        final float power = (float) pid.update(currentPos);

        motor.setPower(power);
        motor2.setPower(power);

//        motor.setPower(-0.4);
//        motor2.setPower(-0.4);

    }

    public static float getAngle() {
        return (currentPos ) / ArmConstants.TicksInRad;

    }

    public static float getWantedAngle() {
        return wantedAngleRads;
    }

    public static float getPower() {
        return (float) motor.getPower();
    }
}
