package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitLEDS;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;

public class OrbitLEDBlinkin {

    private static RevBlinkinLedDriver blinkin;
    private static RevBlinkinLedDriver.BlinkinPattern pattern;
    private static boolean ledOn = false;
    private static float lastChangeTime = 0;
    private static float onTime = 0;
    private static float offTime = 0;

    public static void init(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class,"LED");
    }

    public static void operate(RobotState state, boolean blink) {


        if (ledOn)
        {
            if (GlobalData.currentTime - lastChangeTime > onTime){
                lastChangeTime = GlobalData.currentTime;
                ledOn = false;
            }
        }
        else
        {
            if (GlobalData.currentTime - lastChangeTime > offTime){
                lastChangeTime = GlobalData.currentTime;
                ledOn = true;
            }
        }




        switch (state){
            case TRAVEL:
                pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE;
                break;
            case INTAKE:
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                break;
        }

        if (blink)
        {
            onTime = 0.12f;
            offTime = 0.08f;;
        } else {
            onTime = 10000f;
            offTime = 0f;
        }

        if (ledOn)
            blinkin.setPattern(pattern);
        else
           blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(0));


    }
}
