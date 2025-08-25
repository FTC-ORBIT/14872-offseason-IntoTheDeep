package org.firstinspires.ftc.teamcode.robotSubSystems.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;

public class Intake {
    public static IntakeStates currentState;
    public static float wantedPower;
    public static Servo intakeServo1;
    public static Servo intakeServo2;

    public static void init(HardwareMap hardwareMap, String name, String name2) {
        intakeServo1 = hardwareMap.get(Servo.class, name);
        intakeServo2 = hardwareMap.get(Servo.class, name2);
    }

    public static void operate(IntakeStates state) {
        switch (state) {
            case INTAKE:
                wantedPower = IntakeConstants.intakePower;
                break;

            case STOP:
                wantedPower = IntakeConstants.stoppedPower;
                break;
        }
        if (SubSystemManager.getDeplete()) {
            wantedPower = IntakeConstants.depletePower;

        }

        intakeServo1.setPosition(wantedPower);
        intakeServo2.setPosition(wantedPower);



    }

}
