package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitGamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;

public class OrbitRumble {
    private static float timeOn = 0;
    private static float timeOff = 0;
    private static float lastChangeTime = 0;
    private static boolean rumbleOn = false;
    public static void rumble(Gamepad gamepad, RobotState currentState){

        if (rumbleOn)
        {
            if (GlobalData.currentTime - lastChangeTime > timeOn){
                lastChangeTime = GlobalData.currentTime;
                rumbleOn = false;
            }
        }
        else
        {
            if (GlobalData.currentTime - lastChangeTime > timeOff){
                lastChangeTime = GlobalData.currentTime;
                rumbleOn = true;
            }
        }

        switch (currentState){
            case TRAVEL:
                timeOn = 0;
                timeOff = 10000;
                break;
            case INTAKE:
                if (GlobalData.hasGamePiece){
                    timeOn = 1;
                    timeOff = 1;
                }else {
                    timeOn = 0;
                    timeOff = 10000;
                }
                break;
        }
        if (rumbleOn){
            gamepad.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
        }else {
            gamepad.stopRumble();
        }
    }
}
