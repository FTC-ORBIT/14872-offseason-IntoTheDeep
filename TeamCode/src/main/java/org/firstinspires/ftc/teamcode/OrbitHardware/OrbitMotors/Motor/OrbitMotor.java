package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.OrbitUtils.MotionMagic;
import org.firstinspires.ftc.teamcode.OrbitUtils.MotionProfiles.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.OrbitUtils.PID;
import org.firstinspires.ftc.teamcode.robotData.Constants;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class OrbitMotor {
    private final DcMotorEx motor;
    private final PID motorPID;
    private final MotionMagic motorMotionMagic;
    private DcMotor.RunMode motorRunMode;
    private boolean slaved = false;
    private final DcMotorSimple.Direction direction;
    private final float physicalRatio;
    private final float gearRatio;
    private final float wheelDiameter;
    private Object units;
    private final TrapezoidProfile trapezoidProfile;
    private MotorControlParams params;
    private float peak = 1;
    float prevVel = 0;
    private float trapezoidPresentage = 0.9f;// default 0.9f

    public final boolean isAvailable;

    /**
     * input the wheelDiameter in M,
     * and the systemGearRatio - bigger = power ratio, smaller = vel ratio
     **/

    public OrbitMotor(final HardwareMap hardwareMap, final String name, final DcMotorSimple.Direction direction, final DcMotor.RunMode runMode, final DcMotor.ZeroPowerBehavior zeroPowerBehavior, final MotorControlParams controlParams, final float systemGearRatio, final float wheelDiameter, final Object units) {
        this.motor = hardwareMap.tryGet(DcMotorEx.class, name);
        isAvailable = this.motor != null;

        if (!isAvailable) {
            this.motorPID = null;
            this.motorMotionMagic = null;
            this.physicalRatio = 0;
            this.gearRatio = 0;
            this.wheelDiameter = 0;
            this.direction = null;
            this.trapezoidProfile = null;
            close();
            throw new RuntimeException("Motor not found: " + name);
        }


        this.wheelDiameter = wheelDiameter;
        this.motor.setDirection(direction);
        this.motor.setZeroPowerBehavior(zeroPowerBehavior);
        this.motor.setMode(runMode);
        this.direction = direction;
        motorRunMode = runMode;
        this.gearRatio = (float) (systemGearRatio * motor.getMotorType().getGearing());
        this.physicalRatio = (float) (Math.PI * wheelDiameter * this.gearRatio);
        this.units = units;

        this.params = controlParams;
        this.trapezoidProfile = new TrapezoidProfile();

        motorPID = new PID(controlParams);
        motorMotionMagic = new MotionMagic(controlParams);
    }

    public static OrbitMotor setFactorySettings(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
        return new OrbitMotor(
                hardwareMap,
                name,
                direction,
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.ZeroPowerBehavior.BRAKE,
                MotorControlParams.zero(),
                0,
                0,
                PositionUnits.CM
        );
    }


    public void setUnits(Object units) {
        this.units = units;
    }

    public Object getUnits() {
        return units;
    }


    private void setWanted(final float wanted) {
        motorPID.setWanted(wanted);
        motorMotionMagic.setWanted(wanted);

    }

    public void setPeak(final float peak) {
        this.peak = peak;
    }

    public float getPeak() {
        return peak;
    }

    public void setMotorControlParams(final MotorControlParams params) {
        this.params = params;
        motorMotionMagic.setParams(params);
        motorPID.setParams(params);
    }

    public MotorControlParams getMotorControlParams() {
        return params;
    }

    public void setAccelKick(final float accelKick) {
        motorMotionMagic.setAccelKick(accelKick);
    }

    public void setJerkKick(final float jerkKick) {
        motorMotionMagic.setJerkKick(jerkKick);
    }

    public float getAccelKick() {
        return motorMotionMagic.getAccelKick();
    }

    public float getJerkKick() {
        return motorMotionMagic.getJerkKick();
    }


    public void setTrapezoidPresentage(final float trapezoidPresentage) {
        this.trapezoidPresentage = trapezoidPresentage;
    }

    public float getTrapezoidPresentage() {
        return trapezoidPresentage;
    }


    public void setPower(final MotorControlMode motorControlMode, final float wantedValue, final float arbitraryF) {
        if (!isSlaved() && !checkForMotorFault()) {
            setWanted(wantedValue);
            float power = 0;
            switch (motorControlMode) {
                case PID_POSITION:
                    power = (float) motorPID.update(getCurrentPosition((PositionUnits) units));
                    break;
                case PID_VELOCITY:
                    power = (float) motorPID.update(getVelocity((VelocityUnits) units));
                    break;
                case MOTION_MAGIC_POSITION:
                    power = (float) motorMotionMagic.update(getCurrentPosition((PositionUnits) units), 0);
                    break;
                case MOTION_MAGIC_VELOCITY:
                    power = (float) motorMotionMagic.update(getVelocity((VelocityUnits) units), 0);
                    break;
                case CURRENT:
                    power = (float) motorPID.update(getCurrent((CurrentUnit) units));
                    break;
                case TRAPEZOID_VELOCITY:
                    final float currentVel = getVelocity((VelocityUnits) units);
                    final float deltaVel = wantedValue - currentVel;
                    final float currentAccel = deltaVel / GlobalData.deltaTime;
                    power = trapezoidProfile.velProfile(currentVel, deltaVel, (float) params.maxVel, currentAccel);
                    break;
                case TRAPEZOID_POSITION:
                    final float deltaPos = wantedValue - getCurrentPosition((PositionUnits) units);
                    final float currentVelocity = getVelocity(VelocityUnits.fromPosUnits((PositionUnits) units));
                    final float currentAcceleration = (currentVelocity - prevVel) / GlobalData.deltaTime;
                    prevVel = currentVelocity;
                    power = trapezoidProfile.velProfile(currentVelocity, deltaPos, (float) params.maxVel, currentAcceleration);
                    break;
                case NONE:
                case CUSTOM_CONTROL:
                    power = wantedValue;
                    break;

            }
            power += arbitraryF;
            power = MathFuncs.limit(peak, power);
            motor.setPower(power);
        }
    }


    public void setPower(final float value) {
        if (!isSlaved() && !checkForMotorFault()) motor.setPower(value);
    }

    public float getPower() {
        return Double.isNaN(motor.getPower()) ? 0 : (float) motor.getPower();
    }


    public float getWanted() {
        return motorPID.getWanted();
    }

    public boolean checkForMotorFault() {
        MotorFaults faults = new MotorFaults();
        return faults.checkFaults(this);
    }


    public void resetEncoder() {
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(motorRunMode);
    }


    public void slave(final OrbitMotor masterMotor) {
        slaved = true;
        setPeak(masterMotor.getPeak());
        setPower(masterMotor.getPower());
    }


    public void unSlave() {
        slaved = false;
    }

    public boolean isSlaved() {
        return slaved;
    }

    public boolean inPos(final float ticksForTolerance, final PositionUnits unit) {
        return MathFuncs.inTolerance(getCurrentPosition(unit), getWanted(), ticksForTolerance);
    }

    public boolean inVel(final float velForTolerance, final VelocityUnits unit) {
        return MathFuncs.inTolerance(getVelocity(unit), getWanted(), velForTolerance);
    }


    public void setMotorEnable() {
        motor.setMotorEnable();
    }


    public void setMotorDisable() {
        motor.setMotorDisable();
    }


    public boolean isMotorEnabled() {
        return motor.isMotorEnabled();
    }

    private void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public DcMotorSimple.Direction getDirection() {
        return motor.getDirection();
    }

    public void setVelocity(double angularRate) {
        motor.setVelocity(angularRate);
    }


    public void setVelocity(double angularRate, AngleUnit unit) {
        motor.setVelocity(angularRate, unit);
    }


    public float getVelocity(final VelocityUnits unit) {
        switch (unit) {
            case MM:
                return Double.isNaN(motor.getVelocity()) ? 0 : (float) ((motor.getVelocity() / physicalRatio) * 60);
            case MS:
            default:
                return Double.isNaN(motor.getVelocity()) ? 0 : (float) (motor.getVelocity() / physicalRatio);
            case CMM:
                return Double.isNaN(motor.getVelocity()) ? 0 : (float) ((motor.getVelocity() / physicalRatio / 100) * 60);
            case CMS:
                return Double.isNaN(motor.getVelocity()) ? 0 : (float) (motor.getVelocity() / physicalRatio / 100);
            case DEG:
                return Double.isNaN(motor.getVelocity(AngleUnit.DEGREES)) ? 0 : (float) motor.getVelocity(AngleUnit.DEGREES);
            case RAD:
                return Double.isNaN(motor.getVelocity(AngleUnit.RADIANS)) ? 0 : (float) motor.getVelocity(AngleUnit.RADIANS);
            case TICKS_PER_SECOND:
                return Double.isNaN(motor.getVelocity()) ? 0 : (float) motor.getVelocity();
        }
    }

    public float getCurrentPosition(final PositionUnits unit) {
        switch (unit) {
            case CM:
            default:
                return motor.getCurrentPosition() / physicalRatio;
            case M:
                return (motor.getCurrentPosition() / physicalRatio) * 100;
            case INCH:
                return (motor.getCurrentPosition() / physicalRatio) * 2.54f;
            case TICKS:
                return motor.getCurrentPosition();
            case RADS:
                return (motor.getCurrentPosition() / physicalRatio) / (wheelDiameter / 2) * (float) Math.PI;
            case DEGREES:
                return (float) Math.toDegrees((motor.getCurrentPosition() / physicalRatio) / (wheelDiameter / 2) * (float) Math.PI);
        }
    }


    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }


    public void resetEncoderByCurrentLimitGiven(final float current) {
        if (motor.getCurrent(CurrentUnit.AMPS) > current) {
            resetEncoder();
        }
    }

    public void resetEncoderByCurrentLimitGiven() {
        if (checkForCurrentFault()) resetEncoder();
    }

    public double getCurrentAlert(CurrentUnit unit) {
        return motor.getCurrentAlert(unit);
    }


    private void setCurrentAlert(double current, CurrentUnit unit) {
        motor.setCurrentAlert(current, unit);
    }


    public boolean checkForCurrentFault() {
        return motor.isOverCurrent();
    }


    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    public DcMotorController getController() {
        return motor.getController();
    }

    public int getPortNumber() {
        return motor.getPortNumber();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    public boolean isBusy() {
        return motor.getPower() != 0 || motor.getVelocity() != 0 || motor.getCurrent(CurrentUnit.AMPS) != 0;
    }


    public void setMode(DcMotor.RunMode mode) {
        motorRunMode = mode;
        motor.setMode(mode);
    }


    public DcMotor.RunMode getMode() {
        return motorRunMode;
    }

    public HardwareDevice.Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    public String getDeviceName() {
        return motor.getDeviceName();
    }

    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    public int getVersion() {
        return motor.getVersion();
    }

    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    public void close() {
        motor.close();
    }
}
