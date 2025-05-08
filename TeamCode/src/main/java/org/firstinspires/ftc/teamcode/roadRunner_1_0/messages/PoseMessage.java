package org.firstinspires.ftc.teamcode.roadRunner_1_0.messages;

import com.acmerobotics.roadrunner.Pose2d;

public final class PoseMessage {
    public long timestamp;
    public double x;
    public double y;
    public double heading;

    public PoseMessage(Pose2d pose) {
        timestamp = System.nanoTime();
        x = pose.position.x;
        y = pose.position.y;
        heading = pose.heading.toDouble();
    }
}

