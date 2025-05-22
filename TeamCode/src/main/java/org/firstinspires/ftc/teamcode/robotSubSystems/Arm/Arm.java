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
import org.firstinspires.ftc.teamcode.robotSubSystems.Telescope.Telescope;

public class Arm {
    public static OrbitMotor armMotor;
    public static OrbitMotor armMotor2;
    public static Servo armServo;
    public static float currentAngle;
    public static float wantedAngle;
    public static float servoPos;
    public static ArmStates currentState;
    public static ArmStates lastState;
    public static MotorControlParams armControlParams = new MotorControlParams(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD, ArmConstants.Izone, ArmConstants.KS, ArmConstants.KV);
    public static final float horzAngle = 0f;

    public static void init(HardwareMap hardwareMap, String name, String name2, String name3){
        armMotor = new OrbitMotor(hardwareMap, name, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, armControlParams, ArmConstants.gearRatio, ArmConstants.armWheelDiameter, PositionUnits.RADS);
        armMotor2 = new OrbitMotor(hardwareMap, name2, DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, armControlParams, ArmConstants.gearRatio, ArmConstants.armWheelDiameter, PositionUnits.RADS);
        armServo = hardwareMap.get(Servo.class, name3);
    }
    public static void operate(ArmStates state){
        switch (state){
            case TRAVEL:
                wantedAngle = ArmConstants.travelAngle;
                servoPos = ArmConstants.servoTravelPos;
                break;

            case HIGH_TRAVEL:
                wantedAngle = ArmConstants.highTravelAngle;
                servoPos = ArmConstants.servoHighTravelPos;
                break;

            case INTAKE:
                wantedAngle = ArmConstants.intakeAngle;
                servoPos = ArmConstants.servoIntakePos;
                break;

            case HIGH_CHAMBER:
                wantedAngle = ArmConstants.highChamberAngle;
                servoPos = ArmConstants.servoHighChamberPos;
                break;

            case LOW_BASKET:
                wantedAngle = ArmConstants.lowBasketAngle;
                servoPos = ArmConstants.servoLowBasketPos;
                break;

            case HIGH_BASKET:
                wantedAngle = ArmConstants.highBasketAngle;
                servoPos = ArmConstants.servoHighBasketPos;
                break;

            case CLIMB:
                wantedAngle = ArmConstants.climbAngle;
                servoPos = ArmConstants.servoClimbPos;
                break;

        }
        final float gForce = ArmConstants.KG * MathFuncs.sin(horzAngle - armMotor.getCurrentPosition(PositionUnits.DEGREES)) * Telescope.telescope_distance_LCG;

        armMotor.setPower(MotorControlMode.MOTION_MAGIC_POSITION, wantedAngle,gForce);
        armMotor2.slave(armMotor);


    }
  //  public static float getArbitraryF(){
    //  final float gForce = MathFuncs.sin(0);
    //}
}
