package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Servos;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;

@ServoType(flavor = ServoFlavor.CONTINUOUS)
public class OrbitServoContinuousMode implements Servo {
    private final Servo servo;
    private float wantedPosition = 0;
    public final boolean isAvailable;

    public OrbitServoContinuousMode(final HardwareMap hardwareMap, final String name, final Servo.Direction direction) {
        this.servo = hardwareMap.tryGet(Servo.class, name);
        isAvailable = this.servo != null;
        if (isAvailable) {
            this.servo.setDirection(direction);
        }
    }

    @Override
    public ServoController getController() {
        return isAvailable ? servo.getController() : null;
    }

    @Override
    public int getPortNumber() {
        return isAvailable ? servo.getPortNumber() : -1;
    }

    @Override
    public void setDirection(Servo.Direction direction) {
        if (isAvailable) servo.setDirection(direction);
    }

    @Override
    public Servo.Direction getDirection() {
        return isAvailable ? servo.getDirection() : null;
    }


    // normally  1 - full speed forward, 0 - full speed backward and 0.5 - stop
    // we will change it to 1 - full speed forward, -1 - full speed backward and 0 - stop
    @Override
    public void setPosition(final double position) {
        final float wantedScaledPosition = (float) (position / 2 + 0.5f);
        wantedPosition = wantedScaledPosition;
        if (isAvailable) servo.setPosition(wantedScaledPosition);
    }


    // normally  1 - full speed forward, 0 - full speed backward and 0.5 - stop
    // we will change it to 1 - full speed forward, -1 - full speed backward and 0 - stop
    @Override
    public double getPosition() {
        final float rawPosition = (float) servo.getPosition();
        return isAvailable ? rawPosition * 2 - 1 : -3;
    }

    public boolean inVel(final float positionTolerance) {
        return MathFuncs.inTolerance((float) getPosition(), wantedPosition, positionTolerance);
    }

    @Override
    public void scaleRange(final double min, final double max) {
        if (isAvailable) servo.scaleRange(min, max);
    }


    public HardwareDevice.Manufacturer getManufacturer() {
        return isAvailable ? servo.getManufacturer() : null;
    }


    public String getDeviceName() {
        return isAvailable ? servo.getDeviceName() : "this servo is not available :(((((";
    }


    public String getConnectionInfo() {
        return isAvailable ? servo.getConnectionInfo() : "this servo is not available :(((((";
    }


    public int getVersion() {
        return isAvailable ? servo.getVersion() : -1;
    }


    public void resetDeviceConfigurationForOpMode() {
        if (isAvailable) servo.resetDeviceConfigurationForOpMode();
    }


    public void close() {
        if (isAvailable) servo.close();
    }
}
