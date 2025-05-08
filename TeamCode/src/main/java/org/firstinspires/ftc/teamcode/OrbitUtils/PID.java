package org.firstinspires.ftc.teamcode.OrbitUtils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.MotorControlParams;

public class PID {
    private static final ElapsedTime timer = new ElapsedTime();
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kV = 0;
    public double iZone = 0;

    public double wanted = 0;

    private double integral = 0;

    private double prevError = 0;
    private double prevTime = 0;

    public PID(final MotorControlParams params) {
        this.kP = params.kP;
        this.kI = params.kI;
        this.kD = params.kD;
        this.kV = params.kV;
        this.iZone = params.iZone;
    }

    public void setWanted(final double wanted) {
        this.wanted = wanted;
    }

    public float getWanted(){ return (float) this.wanted;}

    public void setParams(final MotorControlParams params) {
        this.kP = params.kP;
        this.kI = params.kI;
        this.kD = params.kD;
        this.kV = params.kV;
        this.iZone = params.iZone;
    }

    public MotorControlParams getParams() {
        return new MotorControlParams(kP, kI, kD, (float) iZone, kV);
    }

    public double update(final double current) {
        final double currentError = wanted - current;
        final double currentTime = timer.milliseconds() / 1000;
        final double deltaTime = (currentTime - prevTime);

        if (Math.abs(currentError) < iZone) {
            if (Math.signum(currentError) != Math.signum(prevError)) {
                integral = 0;
            } else {
                integral += currentError * deltaTime;
            }
        }

        final double derivative = deltaTime == 0 ? 0 : (currentError - prevError) / deltaTime;

        prevError = currentError;
        prevTime = currentTime;

        final float pidPower = (float) (kP * currentError + kI * integral + kD * derivative);
        return pidPower + kV * Math.signum(pidPower);
    }
}