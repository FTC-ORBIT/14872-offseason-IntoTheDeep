package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainTank;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.PositionUnits;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.OrbitMotor;

public class DriveTrainTank {
    public static final OrbitMotor[] motors = new OrbitMotor[2];
    static ElapsedTime time = new ElapsedTime();
    private static float omega = 0;
    private static float drive = 0;
//    private  static AutoDrive autoDrive = new AutoDrive();

    public static void init(HardwareMap hardwareMap) {
        time.reset();
        motors[0] = OrbitMotor.setFactorySettings(hardwareMap,"leftMotor", DcMotorSimple.Direction.REVERSE);
        motors[1] = OrbitMotor.setFactorySettings(hardwareMap,"rightMotor", DcMotorSimple.Direction.FORWARD);

        drive = 0;
        omega = 0;

    }


    public static void operate(float y_Power, float right_trigger, float left_trigger, Telemetry telemetry, Gamepad gamepad1) {
        float lMotorPower = (y_Power + left_trigger - right_trigger);
        float rMotorPower = (y_Power + right_trigger - left_trigger);
            motors[0].setPower(lMotorPower);
            motors[1].setPower(rMotorPower);
        }


    public static void testMotors(Gamepad gamepad,Telemetry telemetry){  //only for the first time for the configuration
        motors[0].setPower(gamepad.left_stick_y); //leftMotor
        motors[1].setPower(gamepad.right_stick_y); //rightMotor
        telemetry.addData("leftMotor", motors[0].getPower());
        telemetry.addData("rightMotor", motors[1].getPower());
    }
    public static void stop() {
        motors[0].setPower(0);
        motors[1].setPower(0);
    }
    public static void testEncoder(Telemetry telemetry){
        telemetry.addData("leftMotor", motors[0].getCurrentPosition(PositionUnits.TICKS));
        telemetry.addData("rightMotor", motors[1].getCurrentPosition(PositionUnits.TICKS));
    }
    public static void testDeadWheels(Telemetry telemetry){
        telemetry.addData("par",motors[0].getCurrentPosition(PositionUnits.TICKS));
        telemetry.addData("prep",motors[1].getCurrentPosition(PositionUnits.TICKS));
    }
}


