package org.firstinspires.ftc.teamcode.PoseTracker.DriveTrain;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.PositionUnits;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.VelocityUnits;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.OrbitMotor;
import org.firstinspires.ftc.teamcode.OrbitHardware.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitTracker;

public class TankDriveEncodersTracker implements OrbitTracker {
    private final OrbitMotor leftMotor;
    private final OrbitMotor rightMotor;
    private final float physicalUnit; // inches per tick
    private float lastLeftPos = 0;
    private float lastRightPos = 0;
    private float lastHeading = 0;

    public TankDriveEncodersTracker(final HardwareMap hardwareMap, final OrbitMotor leftMotor, final OrbitMotor rightMotor, final float physicalUnit) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.physicalUnit = physicalUnit;
        OrbitGyro.init(hardwareMap);
        reset();
    }

    @Override
    public void reset() {
        lastLeftPos = getLeftPosition();
        lastRightPos = getRightPosition();
        lastHeading = getHeading();
    }

    public float getLeftVelocity() {
        return leftMotor.getVelocity(VelocityUnits.TICKS_PER_SECOND) * physicalUnit;
    }

    public float getRightVelocity() {
        return rightMotor.getVelocity(VelocityUnits.TICKS_PER_SECOND) * physicalUnit;
    }

    @Override
    public Vector getVelocity() {
        return new Vector(getLeftVelocity(), getRightVelocity());
    }

    public float getLeftPosition() {
        return leftMotor.getCurrentPosition(PositionUnits.TICKS) * physicalUnit;
    }

    public float getRightPosition() {
        return rightMotor.getCurrentPosition(PositionUnits.TICKS) * physicalUnit;
    }

    @Override
    public float getHeading() {
        return (float) OrbitGyro.getAngle();
    }

    @Override
    public Pose2D calcDeltaPose() {
        float currentLeftPos = getLeftPosition();
        float currentRightPos = getRightPosition();
        float currentHeading = getHeading();

        float deltaLeft = currentLeftPos - lastLeftPos;
        float deltaRight = currentRightPos - lastRightPos;
        float deltaHeading = currentHeading - lastHeading;

        lastLeftPos = currentLeftPos;
        lastRightPos = currentRightPos;
        lastHeading = currentHeading;


        float deltaX = (deltaLeft + deltaRight) / 2;
        float deltaY = 0;

        return new Pose2D(deltaX, deltaY, deltaHeading);
    }

    @Override
    public Pose2D calcPose() {
       return OrbitPoseTracker.getRobotOrbitPose2D().add(calcDeltaPose());
    }
}
