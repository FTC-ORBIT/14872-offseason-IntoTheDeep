package org.firstinspires.ftc.teamcode.robotSubSystems.Pinch;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pinch {
    public static Servo pinchServo;
    public static float wantedPos = 0f;

    public static void init(HardwareMap hardwareMap, String name){
        pinchServo = hardwareMap.get(Servo.class, name);
    }
    public static void operate(PinchStates state){
        switch(state){
            case CLOSED:
                wantedPos = PinchConstants.closedVal;
                break;

            case OPEN:
                wantedPos = PinchConstants.openVal;
                break;

            case MID_OPEN:
                wantedPos = PinchConstants.midOpenVal;
                break;
        }
        pinchServo.setPosition(wantedPos);
    }
}
