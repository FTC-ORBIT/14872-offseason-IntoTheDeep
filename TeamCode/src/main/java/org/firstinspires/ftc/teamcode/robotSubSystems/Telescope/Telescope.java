package org.firstinspires.ftc.teamcode.robotSubSystems.Telescope;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.MotorControlMode;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.MotorControlParams;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.OrbitMotor;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.PositionUnits;
import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.ArmConstants;

public class Telescope {
    private static OrbitMotor telescopeMotor;
    private static final MotorControlParams telescopeParams = new MotorControlParams(TelescopeConstants.KP, TelescopeConstants.KI, TelescopeConstants.KD, TelescopeConstants.iZone, TelescopeConstants.KS, TelescopeConstants.KV);
    private static float wantedLength;
    public static float currentLength;
    private static TelescopeStates currentState = TelescopeStates.TRAVEL;
    private static TelescopeStates lastState = currentState;
    private static final float zeroPos = 0;


    public static void init(HardwareMap hardwareMap,String name){
        telescopeMotor = new OrbitMotor(hardwareMap, name, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, telescopeParams, 0 /* should be TelescopeConstants.gearRatio */, 0, PositionUnits.M);

        }
    public static void operate(TelescopeStates state){
        switch (state){
            case TRAVEL:
                wantedLength = TelescopeConstants.travelLegnth;
                break;

            case INTAKE:
                wantedLength = TelescopeConstants.intakeLegnth;
                break;

            case LOW_BASKET:
                wantedLength = TelescopeConstants.lowBasketLegnth;
                break;

            case HIGH_BASKET:
                wantedLength = TelescopeConstants.highBasketLegnth;
                break;

            case LOW_CHAMBER:
                wantedLength = TelescopeConstants.lowChamberLegnth;
                break;

            case HIGH_CHAMBER:
                wantedLength = TelescopeConstants.highChamberLegnth;
                break;

            case CLIMB:
                wantedLength = TelescopeConstants.climbLegnth;
                break;
        }
        telescopeMotor.setPower(MotorControlMode.MOTION_MAGIC_POSITION, wantedLength, getArbitaryF());
        telescopeMotor.setPeak(1f);

        lastState = currentState;
        currentLength = telescopeMotor.getCurrentPosition(PositionUnits.M);


    }
    public static float getArbitaryF(){
        final float gForce = MathFuncs.sin(0);
        final float spring = -TelescopeConstants.Kspring * (telescopeMotor.getCurrentPosition(PositionUnits.CM) - TelescopeConstants.travelLegnth);
        return spring + gForce;
    }
}
