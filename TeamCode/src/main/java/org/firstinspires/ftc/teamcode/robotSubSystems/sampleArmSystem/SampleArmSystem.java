package org.firstinspires.ftc.teamcode.robotSubSystems.sampleArmSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.MotorControlMode;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.MotorControlParams;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.OrbitMotor;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.PositionUnits;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.VelocityUnits;
import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.robotData.Constants;
import org.firstinspires.ftc.teamcode.robotSubSystems.OrbitSubSystem;

public class SampleArmSystem implements OrbitSubSystem {
    private static OrbitMotor armMotor;
    private static OrbitMotor armMotor2;
    private static final MotorControlParams params = new MotorControlParams(SampleArmSystemConstants.kp, SampleArmSystemConstants.ki, SampleArmSystemConstants.kd, SampleArmSystemConstants.iZone, SampleArmSystemConstants.ks, SampleArmSystemConstants.kv);
    private static float wantedAngle = 0;
    private static float peak = 0f;
    private static SampleArmSystemStates prevState = SampleArmSystemStates.TRAVEL;

    public static void init(HardwareMap hardwareMap, String name, String name2) {
        armMotor = new OrbitMotor(
                hardwareMap, // this is the hardware map , its for connecting with hardware
                name, // this is the name of the motor
                DcMotorSimple.Direction.FORWARD,// the direction of the motor
                DcMotor.RunMode.RUN_USING_ENCODER,// the DcMotor run mode, we use only - RUN_USING_ENCODER and RUN_WITHOUT_ENCODER
                DcMotor.ZeroPowerBehavior.BRAKE,// the motor zero power behavior
                params, // the parameters for the control
                0,// the gear ratio, bigger = power, smaller = vel
                0, // the .wheel diameter => in Meter units
                PositionUnits.RADS // the unit of the encoder value
        );

        armMotor2 = new OrbitMotor(
                hardwareMap,
                name2,
                DcMotorSimple.Direction.REVERSE,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE,
                params,
                0,
                0,
                PositionUnits.RADS
        );

    }

    public static void operate(SampleArmSystemStates state) {
        switch (state) { // switch for each state, the angle change between states
            case TRAVEL:
                wantedAngle = SampleArmSystemConstants.travelAngleRads;
                peak = 1f;
                break;
            case INTAKE:
                wantedAngle = SampleArmSystemConstants.intakeAngleRads;
                peak = 0.5f;
                break;
            case PLACE:
                peak = 1f;
                wantedAngle = SampleArmSystemConstants.placeAngleRads;
                break;
        }

        armMotor.setPeak(peak);
        armMotor2.setPeak(peak);
        armMotor.setPower(MotorControlMode.MOTION_MAGIC_POSITION, wantedAngle, getArbitraryF()); // activate the motor
        armMotor2.slave(armMotor); // set this motor power to the armMotor power

        if (!prevState.equals(state)) {
            prevState = state;
        }
    }

    public static float getAngle() {
        return (armMotor.getCurrentPosition(PositionUnits.RADS) + armMotor2.getCurrentPosition(PositionUnits.RADS)) / 2; // return the angle of the system
    }

    public static float getArbitraryF() { // this is for negating physicals forces that we don't negate in the normal motion magic like - Elastic force
        // for elastic force its should be =>  -kSpring * (getAngle() - sampleArmSystemConstants.travelAngle)

        return MathFuncs.cos(getAngle()) * SampleArmSystemConstants.kg; // this is for the gravity force
    }


    public static float getAngularVel() {
        return (armMotor.getVelocity(VelocityUnits.RAD) + armMotor2.getVelocity(VelocityUnits.RAD)) / 2;
    }

    public static boolean inPose() {
        return armMotor.inPos(SampleArmSystemConstants.angleTolerance, PositionUnits.RADS) & armMotor2.inPos(SampleArmSystemConstants.angleTolerance, PositionUnits.RADS); // tolerance => we input the allowable error range
    }

    public static float predictAngle() {
        return getAngle() + getAngularVel() * Constants.teleopCodeCycleTime; // because x = v * t
    }

    public static float predictAngle(final float dt) {
        return getAngle() + getAngularVel() * dt;
    }

    public static boolean inPredictedPose(final float dt) {
        return MathFuncs.inTolerance(predictAngle(dt), wantedAngle, SampleArmSystemConstants.angleTolerance);
    }

    public static boolean inPredictedPose() {
        return MathFuncs.inTolerance(predictAngle(), wantedAngle, SampleArmSystemConstants.angleTolerance);
    }

    public static boolean haveFault() {
        return armMotor.checkForMotorFault() || armMotor2.checkForMotorFault(); // check if one of the motor have faults
    }

    public static void setSystemPower(final float power) {
        armMotor.setPower(power);
        armMotor2.setPower(power);
    } // use those kind of methods to tune ks

    public static void tune(Telemetry telemetry) {
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addData("angle", getAngle());
    }
}
