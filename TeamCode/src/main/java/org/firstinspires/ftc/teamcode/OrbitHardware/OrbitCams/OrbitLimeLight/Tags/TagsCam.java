package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitCams.OrbitLimeLight.Tags;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose3D;

public interface TagsCam {
    void init(final HardwareMap hardwareMap, final String cameraName,final int pipeline);
    Pose2D robotPose2D();
    Pose3D robotPose3D();
    boolean isTargetValid();
    void stop();
}
