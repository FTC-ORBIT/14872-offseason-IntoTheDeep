package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Servos;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OrbitServo {


    public static Servo OrbitServoFactory(final HardwareMap hardwareMap, final String name, final Servo.Direction direction, final ServoMode servoMode) {
        switch (servoMode) {
            default:
            case STANDARD:
               return new OrbitServoStandardMode(hardwareMap, name, direction);
            case CONTINUOUS:
                return new OrbitServoContinuousMode(hardwareMap, name,direction);
        }
    }
}
