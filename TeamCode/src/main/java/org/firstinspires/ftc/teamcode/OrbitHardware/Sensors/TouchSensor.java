package org.firstinspires.ftc.teamcode.OrbitHardware.Sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotData.Constants;

public class TouchSensor {

    private final com.qualcomm.robotcore.hardware.TouchSensor touchSensor;

    public TouchSensor(final HardwareMap hardwareMap, final String name) {
        touchSensor = hardwareMap.tryGet(com.qualcomm.robotcore.hardware.TouchSensor.class, name);
    }

    public float getValue() {
        return touchSensor != null ? (float) touchSensor.getValue() : Constants.INF;
    }

    public boolean getState() {
        return touchSensor != null && touchSensor.isPressed();
    }
}