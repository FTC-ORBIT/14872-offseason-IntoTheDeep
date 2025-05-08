package org.firstinspires.ftc.teamcode.OrbitHardware.Sensors;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;

public class OrbitPinPoint {
    private final GoBildaPinpointDriver pinpointDriver;

    public OrbitPinPoint(final HardwareMap hardwareMap, final String name) {
        this.pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, name);
    }

    public Pose2D getPose() {
        return new Pose2D((float) pinpointDriver.getPosX(), (float) pinpointDriver.getPosY(), (float) pinpointDriver.getHeading());
    }

    public float getX() {
        return (float) pinpointDriver.getPosX();
    }

    public float getY() {
        return (float) pinpointDriver.getPosY();
    }

    public float getHeading() {
        return (float) pinpointDriver.getHeading();
    }

}
