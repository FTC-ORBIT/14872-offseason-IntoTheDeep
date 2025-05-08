package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitCams.OrbitLimeLight.Tags;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose3D;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;

public class LimeMegaTag2 implements TagsCam{
    private final Pose3D camPose = new Pose3D(0,0,0,0,0,0); // TODO: Set the camera pose
    private Limelight3A limelight3A;
    @Override
    public void init(final HardwareMap hardwareMap, final String cameraName,final int pipeline) {
        limelight3A = hardwareMap.get(Limelight3A.class, cameraName);
        limelight3A.pipelineSwitch(pipeline);
        limelight3A.start();
    }

    @Override
    public Pose2D robotPose2D() {
        update();
        LLResult result = limelight3A.getLatestResult();
        if (result.isValid()) {
            Pose3D robotPose3D = Pose3D.fromSDKPose3D(result.getBotpose_MT2());
            return Pose2D.from3D(robotPose3D);
        }
        return null;
    }

    @Override
    public Pose3D robotPose3D() {
        update();
        LLResult result = limelight3A.getLatestResult();
        if (result.isValid()) {
            return Pose3D.fromSDKPose3D(result.getBotpose_MT2());
        }
        return null;
    }

    @Override
    public boolean isTargetValid() {
        LLResult result = limelight3A.getLatestResult();
        return result.isValid();
    }

    @Override
    public void stop() {
    limelight3A.stop();
    }

    private void update(){
       limelight3A.updateRobotOrientation(Angle.radToDeg(OrbitPoseTracker.getHeading()));
    }
}
