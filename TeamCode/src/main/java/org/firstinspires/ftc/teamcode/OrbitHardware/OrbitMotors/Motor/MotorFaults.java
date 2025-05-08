package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class MotorFaults {
    public enum Type {
        STATOR, SENSOR
    }

    private static final int sensorFaultThreshold = 5;

    private float lastMotorPos = 0;
    private int equalPositionsCounter = 0;
    private boolean haveFault = false;
    public Type faultType = null;

    public boolean checkFaults(OrbitMotor motor) {
        if (!GlobalData.inCompetition) {

            boolean currentFault = motor.checkForCurrentFault();

            if (currentFault) {
                faultType = Type.STATOR;
                haveFault = true;
                motor.setMotorDisable();
            }

            if (motor.isBusy() && motor.getCurrentPosition(PositionUnits.TICKS) == lastMotorPos) {
                equalPositionsCounter++;
            } else {
                equalPositionsCounter = 0;
            }

            if (equalPositionsCounter >= sensorFaultThreshold && motor.isBusy()) {
                equalPositionsCounter = 0;
                faultType = Type.SENSOR;
                haveFault = true;
                motor.setMotorDisable();
                throw new RuntimeException(faultType.name() + " fault");
            }

            if (!currentFault && equalPositionsCounter < sensorFaultThreshold) {
                haveFault = false;
                faultType = null;
                motor.setMotorEnable();
            }

            lastMotorPos = motor.getCurrentPosition(PositionUnits.TICKS);
        }

        return haveFault;
    }
}
