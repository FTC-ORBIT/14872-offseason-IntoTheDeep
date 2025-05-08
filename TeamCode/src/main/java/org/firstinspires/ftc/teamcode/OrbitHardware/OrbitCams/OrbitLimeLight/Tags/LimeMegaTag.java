package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitCams.OrbitLimeLight.Tags;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose3D;

public class LimeMegaTag implements TagsCam{
    private static final Pose3D camPose = new Pose3D(0,0,0,0,0,0); // TODO: Set the camera pose
    private Limelight3A limelight;
    @Override
    public void init(final HardwareMap hardwareMap, final String cameraName,final int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, cameraName);
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    @Override
    public Pose2D robotPose2D() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()){
           final Pose3D robotPose3D = Pose3D.fromSDKPose3D(result.getBotpose());
            return Pose2D.from3D(robotPose3D);
        }
        return null;
    }


    @Override
    public Pose3D robotPose3D() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()){
            return Pose3D.fromSDKPose3D(result.getBotpose());
        }
        return null;
    }



    @Override
    public boolean isTargetValid() {
        LLResult result = limelight.getLatestResult();
        return result.isValid();
    }

    @Override
    public void stop() {
    limelight.stop();
    }


}
