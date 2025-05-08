package org.firstinspires.ftc.teamcode.OrbitHardware.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.robotData.Constants;

public class OrbitIr {
    private final IrSeekerSensor irSeekerSensor;

    public OrbitIr(final HardwareMap hardwareMap, final String name) {
        irSeekerSensor = hardwareMap.tryGet(IrSeekerSensor.class, name);
    }

    public float getAngle() {
        return irSeekerSensor != null ? Angle.degToRad((float) irSeekerSensor.getAngle()) : Constants.INF;
    }

    public float getSignalDetected() {
        return irSeekerSensor != null ? (float) irSeekerSensor.getStrength() : Constants.INF;
    }

    public boolean isSignalDetected() {
        return irSeekerSensor != null && irSeekerSensor.signalDetected();
    }

    public boolean isSignalDetected(float threshold) {
        return irSeekerSensor != null && irSeekerSensor.signalDetected() && irSeekerSensor.getStrength() > threshold;
    }

    public boolean isSignalDetected(float threshold, float angle) {
        return irSeekerSensor != null && irSeekerSensor.signalDetected() && irSeekerSensor.getStrength() > threshold && irSeekerSensor.getAngle() == angle;
    }

}
