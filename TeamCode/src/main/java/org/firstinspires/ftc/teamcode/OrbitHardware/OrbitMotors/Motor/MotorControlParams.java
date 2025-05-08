package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor;

import org.firstinspires.ftc.teamcode.robotData.Constants;

public class MotorControlParams {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double iZone;
    public final double kS;
    public final double kV;
    public final double kA;
    public final double kJ;
    public final double maxVel;
    public final double maxAcc;
    public final double maxJerk;

    public MotorControlParams(final double kP, final double kI, final double kD, final double iZone, final double kS, final double kV){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iZone = iZone;
        this.kS = kS;
        this.kV = kV;
        this.kA = 0;
        this.kJ = 0;
        this.maxVel = 0;
        this.maxAcc = 0;
        this.maxJerk = 0;
    }


    public MotorControlParams(final double kP, final double kI, final double kD, final double iZone, final double kS, final double kV, final double kA){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iZone = iZone;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kJ = 0;
        this.maxVel = 0;
        this.maxAcc = 0;
        this.maxJerk = 0;
    }


    public MotorControlParams(final double kP, final double kI, final double kD, final double iZone, final double kS, final double kV, final double kA,final double kJ){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iZone = iZone;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kJ = kJ;
        this.maxVel = 0;
        this.maxAcc = 0;
        this.maxJerk = 0;
    }

    public MotorControlParams(final double kP, final double kI, final double kD, final double iZone, final double kS, final double kV, final double kA,final double kJ,final double maxVel){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iZone = iZone;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kJ = kJ;
        this.maxVel = maxVel;
        this.maxAcc = 0;
        this.maxJerk = 0;
    }


    public MotorControlParams(final double kP, final double kI, final double kD, final double iZone, final double kS, final double kV, final double kA,final double kJ,final double maxVel,final double maxAcc){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iZone = iZone;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kJ = kJ;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
        this.maxJerk = 0;
    }

    public MotorControlParams(final double kP, final double kI, final double kD, final double iZone, final double kS, final double kV, final double kA,final double kJ,final double maxVel,final double maxAcc,final double maxJerk){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iZone = iZone;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kJ = kJ;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
        this.maxJerk = maxJerk;
    }


    public MotorControlParams(final double kp, final double ki, final double kd, final float kv, final double iZone){
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
        this.iZone = iZone;
        this.kS = 0;
        this.kV = kv;
        this.kA = 0;
        this.kJ = 0;
        this.maxVel = 0;
        this.maxAcc = 0;
        this.maxJerk = 0;
    }
    public static MotorControlParams zero(){
        return new MotorControlParams(0, 0, 0, 0, 0);
    }

    public static MotorControlParams INF(){
        return new MotorControlParams(Constants.INF, Constants.INF, Constants.INF, Constants.INF, Constants.INF);
    }// this is a bad idea, never use this, it will break everything,but this is funny so I will keep it :)

    public static MotorControlParams epsilon(){
        return new MotorControlParams(Constants.epsilon, Constants.epsilon, Constants.epsilon, Constants.epsilon, Constants.epsilon);
    }// might be useful for some cases

}
