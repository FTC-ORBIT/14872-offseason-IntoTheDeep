package org.firstinspires.ftc.teamcode.robotSubSystems.Telescope;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.MotorControlMode;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.MotorControlParams;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.OrbitMotor;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.PositionUnits;
import org.firstinspires.ftc.teamcode.robotData.Constants;

public class Telescope {
    private static OrbitMotor telescopeMotor;
    private static MotorControlParams telescopeParams = new MotorControlParams(TelescopeConstants.KP, TelescopeConstants.KI, TelescopeConstants.KD, TelescopeConstants.iZone, TelescopeConstants.KS, TelescopeConstants.KV);
    private static float wantedLength;
    private static TelescopeStates currentState = TelescopeStates.TRAVEL;
    private static TelescopeStates lastState = currentState;
    private static float zeroPos = 0;

    public static void init(HardwareMap hardwareMap,String name){
        telescopeMotor = new OrbitMotor(hardwareMap, name, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.ZeroPowerBehavior.FLOAT, telescopeParams, 0 /* should be TelescopeConstants.gearRatio */, 0, PositionUnits.CM);


        }
    public static void operate(TelescopeStates state){
        switch (state){
            case TRAVEL:
                wantedLength = TelescopeConstants.travelPos;
                break;

            case INTAKE:
                wantedLength = TelescopeConstants.intakePos;
                break;

            case LOW_BASKET:
                wantedLength = TelescopeConstants.lowBasketPose;
                break;

            case HIGH_BASKET:
                wantedLength = TelescopeConstants.highBasketPos;
                break;

            case LOW_CHAMBER:
                wantedLength = TelescopeConstants.lowChamberPos;
                break;
            case HIGH_CHAMBER:
                wantedLength = TelescopeConstants.highChamberPos;
        }
        telescopeMotor.setPower(MotorControlMode.MOTION_MAGIC_VELOCITY, wantedLength, 0);

        telescopeMotor.getCurrentPosition(PositionUnits.CM);

        lastState = currentState;
    }
}
