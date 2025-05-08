package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Servos;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;

@ServoType(flavor = ServoFlavor.STANDARD)
public class OrbitServoStandardMode implements Servo {

    private final Servo servo;
    private float wantedPosition = 0;
    public final boolean isAvailable;

    public OrbitServoStandardMode(final HardwareMap hardwareMap, final String name, final Servo.Direction direction) {
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
    public void setDirection(final Servo.Direction direction) {
        if (isAvailable) servo.setDirection(direction);
    }

    @Override
    public Servo.Direction getDirection() {
        if (isAvailable) return servo.getDirection();
        else return null;
    }

    @Override
    public void setPosition(final double position) {
        wantedPosition = (float) position;
        if (isAvailable) servo.setPosition(position);
    }

    @Override
    public double getPosition() {
        if (isAvailable) return servo.getPosition();
        else return -1;
    }

    public boolean inPose(final float positionTolerance) {
        return MathFuncs.inTolerance((float) getPosition(), wantedPosition, positionTolerance);
    }

    @Override
    public void scaleRange(final double min, final double max) {
      if (isAvailable) servo.scaleRange(min, max);
    }

    @Override
    public HardwareDevice.Manufacturer getManufacturer() {
        return servo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return isAvailable ? servo.getDeviceName() : "this servo is not available :(((((";
    }

    @Override
    public String getConnectionInfo() {
        return isAvailable ? servo.getConnectionInfo() : "this servo is not available :(((((";
    }

    @Override
    public int getVersion() {
        return isAvailable ? servo.getVersion() : 0 ;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
      if (isAvailable) servo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
      if (isAvailable) servo.close();
    }
}
