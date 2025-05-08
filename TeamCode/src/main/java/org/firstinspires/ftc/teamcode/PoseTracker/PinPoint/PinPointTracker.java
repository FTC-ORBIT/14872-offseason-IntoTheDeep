package org.firstinspires.ftc.teamcode.PoseTracker.PinPoint;
import org.firstinspires.ftc.teamcode.OrbitHardware.Sensors.OrbitPinPoint;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitTracker;

public class PinPointTracker implements OrbitTracker {
    private final OrbitPinPoint pinPoint;

    public PinPointTracker(final OrbitPinPoint pinPoint) {
        this.pinPoint = pinPoint;
    }

    @Override
    public void reset() {
        // No reset needed for PinPoint
    }

    @Override
    public Vector getVelocity() {
        return null;
    }

    @Override
    public float getHeading() {
        return pinPoint.getHeading();
    }

    @Override
    public Pose2D calcDeltaPose() {
        return null;
    }

    @Override
    public Pose2D calcPose() {
        return pinPoint.getPose();
    }
}
