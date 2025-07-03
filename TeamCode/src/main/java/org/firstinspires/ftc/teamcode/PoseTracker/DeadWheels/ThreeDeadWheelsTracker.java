package org.firstinspires.ftc.teamcode.PoseTracker.DeadWheels;


import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.PositionUnits;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.VelocityUnits;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.OrbitMotor;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitTracker;

public class ThreeDeadWheelsTracker implements DeadWheelsTracker {
    private final OrbitMotor leftParWheel;
    private final OrbitMotor rightParWheel;
    private final OrbitMotor perpWheel;

    private final float trackWidth;
    private final float perpOffset;
    private final float positionPhysicalUnit;
    private final float headingPhysicalUnit;

    private float lastLeftPos = 0;
    private float lastRightPos = 0;
    private float lastPerpPos = 0;

    private final PositionUnits positionUnit = PositionUnits.TICKS;
    private final VelocityUnits velocityUnit = VelocityUnits.TICKS_PER_SECOND;

    public ThreeDeadWheelsTracker(final OrbitMotor leftParWheel, final OrbitMotor rightParWheel, final OrbitMotor perpWheel, final float trackWidth, final float perpOffset, final float positionPhysicalUnit, final float headingPhysicalUnit) {
        this.leftParWheel = leftParWheel;
        this.rightParWheel = rightParWheel;
        this.perpWheel = perpWheel;
        this.trackWidth = trackWidth;
        this.perpOffset = perpOffset;
        this.positionPhysicalUnit = positionPhysicalUnit;
        this.headingPhysicalUnit = headingPhysicalUnit;

        reset();
    }

    @Override
    public void reset() {
        lastLeftPos = getLeftParPosition();
        lastRightPos = getRightParPosition();
        lastPerpPos = getPerpPosition();
    }

    @Override
    public float getParVelocity() {
        return (getLeftParVelocity() + getRightParVelocity()) / 2f;
    }

    @Override
    public float getPerpVelocity() {
        return perpWheel.getVelocity(velocityUnit) * positionPhysicalUnit;
    }

    private float getLeftParVelocity() {
        return leftParWheel.getVelocity(velocityUnit) * positionPhysicalUnit;
    }

    private float getRightParVelocity() {
        return rightParWheel.getVelocity(velocityUnit) * positionPhysicalUnit;
    }

    @Override
    public Vector getVelocity() {
        return new Vector(getParVelocity(), getPerpVelocity());
    }

    private float getLeftParPosition() {
        return leftParWheel.getCurrentPosition(positionUnit);
    }

    private float getRightParPosition() {
        return rightParWheel.getCurrentPosition(positionUnit);
    }

    @Override
    public float getParPosition() {
        return (getLeftParPosition() + getRightParPosition()) / 2f;
    }

    @Override
    public float getPerpPosition() {
        return perpWheel.getCurrentPosition(positionUnit);
    }

    @Override
    public float getHeading() {
        final float deltaLeft = getLeftParPosition() - lastLeftPos;
        final float deltaRight = getRightParPosition() - lastRightPos;
        return ((deltaRight - deltaLeft) * headingPhysicalUnit) / trackWidth;
    }

    @Override
    public Pose2D calcDeltaPose() {
        final float currentLeft = getLeftParPosition();
        final float currentRight = getRightParPosition();
        final float currentPerp = getPerpPosition();

        final float deltaLeft = (currentLeft - lastLeftPos) * positionPhysicalUnit;
        final float deltaRight = (currentRight - lastRightPos) * positionPhysicalUnit;
        final float deltaPerp = (currentPerp - lastPerpPos) * positionPhysicalUnit;

        final float deltaHeading = (deltaRight - deltaLeft) / trackWidth;
        final float deltaForward = (deltaLeft + deltaRight) / 2f;
        final float deltaStrafe = deltaPerp - (perpOffset * deltaHeading);


        final float avgHeading = deltaHeading / 2f;
        final Vector deltaVector = new Vector(deltaForward, deltaStrafe).rotate(avgHeading);

        lastLeftPos = currentLeft;
        lastRightPos = currentRight;
        lastPerpPos = currentPerp;

        return new Pose2D(deltaVector, deltaHeading);
    }

    @Override
    public Pose2D calcPose() {
      return  OrbitPoseTracker.getRobotOrbitPose2D().add(calcDeltaPose());
    }
}
