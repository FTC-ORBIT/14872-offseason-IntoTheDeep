package org.firstinspires.ftc.teamcode.PoseTracker.DriveTrain;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.OrbitMotor;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.PositionUnits;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.VelocityUnits;
import org.firstinspires.ftc.teamcode.OrbitHardware.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitTracker;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;

public class OmniDriveEncodersTracker implements OrbitTracker {
    private final OrbitMotor lf;
    private final OrbitMotor rf;
    private final OrbitMotor lb;
    private final OrbitMotor rb;
    private final float physicalUnit; // inches per tick
    private final VelocityUnits velocityUnit = VelocityUnits.TICKS_PER_SECOND;
    private final PositionUnits positionUnit = PositionUnits.TICKS;

    private float lastLFPos = 0;
    private float lastRFPos = 0;
    private float lastLBPos = 0;
    private float lastRBPos = 0;
    private float lastHeading = 0;

    public OmniDriveEncodersTracker(final HardwareMap hardwareMap, final OrbitMotor lf, final OrbitMotor rf, final OrbitMotor lb, final OrbitMotor rb, final float physicalUnit) {
        this.lf = lf;
        this.rf = rf;
        this.lb = lb;
        this.rb = rb;
        this.physicalUnit = physicalUnit;
        OrbitGyro.init(hardwareMap);
        reset(); // initialize state
    }


    @Override
    public void reset() {
        this.lastLFPos = getLeftFrontPosition();
        this.lastRFPos = getRightFrontPosition();
        this.lastLBPos = getLeftBackPosition();
        this.lastRBPos = getRightBackPosition();
    }

    public float getLeftFrontVelocity() {
        return lf.getVelocity(velocityUnit) * physicalUnit;
    }

    public float getRightFrontVelocity() {
        return rf.getVelocity(velocityUnit) * physicalUnit;
    }

    public float getLeftBackVelocity() {
        return lb.getVelocity(velocityUnit) * physicalUnit;
    }

    public float getRightBackVelocity() {
        return rb.getVelocity(velocityUnit) * physicalUnit;
    }



    @Override
    public Vector getVelocity() {
        return DrivetrainOmni.getVelocity_FieldCS();
    }

    public float getLeftFrontPosition() {
        return lf.getCurrentPosition(positionUnit) * physicalUnit;
    }

    public float getRightFrontPosition() {
        return rf.getCurrentPosition(positionUnit) * physicalUnit;
    }

    public float getLeftBackPosition() {
        return lb.getCurrentPosition(positionUnit) * physicalUnit;
    }

    public float getRightBackPosition() {
        return rb.getCurrentPosition(positionUnit) * physicalUnit;
    }

    @Override
    public float getHeading() {
        return (float) OrbitGyro.getAngle();
    }


    @Override
    public Pose2D calcDeltaPose() {
        final float currentLF = getLeftFrontPosition();
        final float currentRF = getRightFrontPosition();
        final float currentLB = getLeftBackPosition();
        final float currentRB = getRightBackPosition();
        final float currentHeading = getHeading();

        final float deltaLF = currentLF - lastLFPos;
        final float deltaRF = currentRF - lastRFPos;
        final float deltaLB = currentLB - lastLBPos;
        final float deltaRB = currentRB - lastRBPos;
        final float deltaHeading = currentHeading - lastHeading;


        final float avgHeading = lastHeading + deltaHeading / 2f;


        lastLFPos = currentLF;
        lastRFPos = currentRF;
        lastLBPos = currentLB;
        lastRBPos = currentRB;
        lastHeading = currentHeading;


        final float deltaX = (deltaLF + deltaRF + deltaLB + deltaRB) / 4f;  // Forward/backward
        final float deltaY = (deltaLF - deltaRF + deltaLB - deltaRB) / 4f;  // Strafe (sideways)



        final Vector delta = new Vector(deltaX, deltaY).rotate(avgHeading);

        return new Pose2D(delta, deltaHeading);
    }


    @Override
    public Pose2D calcPose() {
       return OrbitPoseTracker.getRobotOrbitPose2D().add(calcDeltaPose());
    }
}