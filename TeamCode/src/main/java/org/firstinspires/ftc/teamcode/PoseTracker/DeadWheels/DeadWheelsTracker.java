package org.firstinspires.ftc.teamcode.PoseTracker.DeadWheels;


import org.firstinspires.ftc.teamcode.PoseTracker.OrbitTracker;

public interface DeadWheelsTracker extends OrbitTracker {
    float getParVelocity();
    float getPerpVelocity();

    float getParPosition();
    float getPerpPosition();
}
