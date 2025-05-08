package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitLEDS;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.sampleArmSystem.SampleArmSystem;

public class OrbitLEDServoVersion {

    private static Servo LEDservo;
    private static float color;
    private static boolean ledOn = false;
    private static float lastChangeTime = 0;
    private static float onTime = 0;
    private static float offTime = 0;

    private static boolean leftBumper;
    private static boolean rightBumper;
    private static boolean leftDPade;
    private static boolean rightDPade;


    public static void init(HardwareMap hardwareMap, String name){
        LEDservo = hardwareMap.get(Servo.class, name);
        lastChangeTime = GlobalData.currentTime;
    }

    public static void operate(RobotState state, boolean blink){
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
                color = 0.6f;
                break;
            case INTAKE:
                color = 0.4f;
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
            LEDservo.setPosition(color);
        else
            LEDservo.setPosition(0);


        if (SampleArmSystem.haveFault()) LEDservo.setPosition(0.25);
    }

    public static void test(Gamepad gamepad1, Telemetry telemetry){
        if (!leftBumper && gamepad1.left_bumper){
            color += 0.05f;
        } else if (!rightBumper && gamepad1.right_bumper){
            color -= 0.05f;
        } else if (!leftDPade && gamepad1.dpad_left){
            color += 0.01f;
        } else if (!rightDPade && gamepad1.dpad_right){
            color -= 0.01f;
        }
        leftBumper = gamepad1.left_bumper;
        rightBumper = gamepad1.right_bumper;
        leftDPade = gamepad1.dpad_left;
        rightDPade = gamepad1.dpad_right;
        LEDservo.setPosition(color);
        telemetry.addData( "color val" , color);
        telemetry.update();
    }

}


//red = 0.25
//yellow = 0.3
//green = 0.4
//light blue = 0.48
//blue = 0.6
//purple = 0.7