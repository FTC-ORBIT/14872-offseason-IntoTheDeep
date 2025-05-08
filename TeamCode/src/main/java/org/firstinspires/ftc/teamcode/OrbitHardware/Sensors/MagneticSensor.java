package org.firstinspires.ftc.teamcode.OrbitHardware.Sensors;


import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.robotData.Constants;

public class MagneticSensor {
    private final TouchSensor magneticSensor;


    public MagneticSensor(final HardwareMap hardwareMap, final String name){
        magneticSensor = hardwareMap.tryGet(TouchSensor.class, name);
    }

    public float getValue(){
        return magneticSensor != null ? (float) magneticSensor.getValue() : Constants.INF;
    }

    public boolean getState(){
        return magneticSensor != null && magneticSensor.isPressed();
    }
}