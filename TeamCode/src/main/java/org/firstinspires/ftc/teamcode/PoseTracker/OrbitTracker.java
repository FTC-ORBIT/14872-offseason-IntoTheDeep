package org.firstinspires.ftc.teamcode.PoseTracker;

import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;

public interface OrbitTracker {
    void reset();

    Vector getVelocity();
    float getHeading();

    Pose2D calcDeltaPose();

    Pose2D calcPose();

}
