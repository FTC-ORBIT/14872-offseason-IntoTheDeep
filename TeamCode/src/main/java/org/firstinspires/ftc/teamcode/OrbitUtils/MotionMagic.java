package org.firstinspires.ftc.teamcode.OrbitUtils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.MotorControlParams;
import org.firstinspires.ftc.teamcode.OrbitUtils.MotionProfiles.PolygonProfile;
import org.firstinspires.ftc.teamcode.OrbitUtils.MotionProfiles.SCurveProfile;
import org.firstinspires.ftc.teamcode.OrbitUtils.MotionProfiles.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.robotData.Constants;

public class MotionMagic {
    private final ElapsedTime timer = new ElapsedTime();

    // the K's
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kV = 0;
    public double iZone = 0;
    public double kS = 0;
    public double kA = 0;
    public double kJ = 0;

    // the power we calculate
    public double pS = 0;
    public double vW = 0;
    public double aW = 0;
    public double jW = 0;
    public double pV = 0;
    public double pA = 0;
    public double pJ = 0;
    public double pPID = 0;


    // the previous values
    private double prevError = 0;
    private double prevTime = 0;
    public double aWPrev = 0;
    public double posWPrev = 0;
    public double vWPrev = 0;

    // more  values
    public double vMax = 0;
    public double aMax = 0;
    public double jMax = 0;
    public double target;
    public double prevTarget = Constants.INF;
    public double posW = 0;
    private double integral = 0;
    public double accelKick = 1; // only for starting the motion
    public double jerkKick = 10; // only for starting the motion
    public double startMotionCurrent = 0;
    public double prevDeltaTime = 0;
    public TrapezoidProfile trapezoidProfile;
    public SCurveProfile sCurveProfile;
    public PolygonProfile polygonProfile;

    public MotionMagic(final MotorControlParams params) {
        this.kP = params.kP;
        this.kI = params.kI;
        this.kD = params.kD;
        this.iZone = params.iZone;
        this.kS = params.kS;
        this.kV =  params.kV;
        this.kA = params.kA;
        this.kJ = params.kJ;
        this.vMax = params.maxVel;
        this.aMax = params.maxAcc;
        this.jMax = params.maxJerk;

        trapezoidProfile = new TrapezoidProfile();
        sCurveProfile = new SCurveProfile();
        polygonProfile = new PolygonProfile();

        sCurveProfile.setProfileParams((float) vMax, (float) aMax, (float) jMax);
        polygonProfile.setProfileParams((float) vMax, (float) aMax, (float) jMax);

        timer.reset();
    }

    public void setWanted(final double wanted) {
        this.target = wanted;
    }

    public float getWanted() {
        return (float) target;
    }

    public void setAccelKick(final double accelKick) {
        this.accelKick = accelKick;
    }

    public void setJerkKick(final double jerkKick) {
        this.jerkKick = jerkKick;
    }

    public float getAccelKick(){
        return (float) accelKick;
    }

    public float getJerkKick(){
        return (float) jerkKick;
    }

    public void setParams(final MotorControlParams params) {
        this.kP = params.kP;
        this.kI = params.kI;
        this.kD = params.kD;
        this.iZone = params.iZone;
        this.kS = params.kS;
        this.kV =  params.kV;
        this.kA = params.kA;
        this.kJ = params.kJ;
        this.vMax = params.maxVel;
        this.aMax = params.maxAcc;
        this.jMax = params.maxJerk;
        sCurveProfile.setProfileParams((float) vMax, (float) aMax, (float) jMax);
        polygonProfile.setProfileParams((float) vMax, (float) aMax, (float) jMax);
    }

    public MotorControlParams getParams() {
        return new MotorControlParams(kP, kI, kD, iZone, kS, kV, kA, kJ ,vMax, aMax, jMax);
    }

    public double update(final double current) {
        final double currentTime = timer.seconds();
        final double deltaTime = currentTime - prevTime;

        boolean targetChanged = target != prevTarget || prevTarget == Constants.INF;
        if (targetChanged || (vW == 0 && prevDeltaTime == 0)) {
            jW = Math.copySign(jerkKick, target - current);
            aW = Math.copySign(accelKick, target - current);
            startMotionCurrent = current;
        } else {
            jW = deltaTime != 0 ? (aW - aWPrev) / deltaTime : 0;
            aW = aWPrev + jW * deltaTime;
        }

        jW = MathFuncs.limit(jMax == 0 ? Constants.INF : (float) jMax, (float) jW);
        aW = MathFuncs.limit( aMax == 0 ? Constants.INF : (float) aMax, (float) aW);


        vW = MathFuncs.min(
                vWPrev + aW * deltaTime,
                vMax != 0 ? vMax : Constants.INF ,
                Math.sqrt(Math.abs(2 * aW * Math.abs(target - current))),

                vMax != 0 ?
                        MathFuncs.twoPointsLinear(
                                new Vector((float) startMotionCurrent, (float) vMax),
                                new Vector((float) target, 0), (float) current)
                        : Constants.INF ,

                vMax != 0 ? trapezoidProfile.velProfile((float) vWPrev, (float) (target - current), (float) vMax, (float) aW)
                        : Constants.INF,
                sCurveProfile.velProfile((float) deltaTime, (float) target, (float) current),
                polygonProfile.velProfile((float) deltaTime, (float) target, (float) current)
        );


        posW = posWPrev + vWPrev * deltaTime + 0.5 * aWPrev * (deltaTime * deltaTime);

        pS = Math.signum(vW) * kS;
        pV = vW * kV;
        pA = aW * kA;
        pJ = jW * kJ;

        // pid power calc:
        final double currentError = posW - current;
        if (Math.abs(currentError) < iZone) {
            if (Math.signum(currentError) != Math.signum(prevError)) {
                integral = 0;
            } else {
                integral += currentError * deltaTime;
            }
        }

        final double derivative = deltaTime == 0 ? 0 : (currentError - prevError) / deltaTime;

        pPID = kP * currentError + kI * integral + kD * derivative;


        prevError = currentError;
        prevTime = currentTime;
        aWPrev = aW;
        posWPrev = posW;
        vWPrev = vW;
        prevTarget = target;
        prevDeltaTime = deltaTime;

        return pS + pV + pA + pJ + pPID;
    }
}