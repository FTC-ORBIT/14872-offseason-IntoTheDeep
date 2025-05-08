package org.firstinspires.ftc.teamcode.OrbitHardware.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotData.Constants;

public class Potentiometer {

    private AnalogInput potentiometer;

    public Potentiometer(final HardwareMap hardwareMap, final String name) {
        potentiometer = hardwareMap.tryGet(AnalogInput.class, name);
    }

    public float getVolt(){
        return potentiometer != null ? (float) potentiometer.getVoltage() : Constants.INF;
    }
}
