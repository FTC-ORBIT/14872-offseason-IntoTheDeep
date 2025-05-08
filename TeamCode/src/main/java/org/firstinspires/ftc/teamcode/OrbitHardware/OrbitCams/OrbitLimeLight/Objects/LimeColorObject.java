package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitCams.OrbitLimeLight.Objects;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose3D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;

public class LimeColorObject {
    private Limelight3A limelight;
    private final Pose3D camPose = new Pose3D(0,0,0,0,0,0); // TODO Set this to the actual camera pose
    private final float ObjectHeight = 0; // TODO Set this to the actual object height in Inches

    public void init(final HardwareMap hardwareMap, final String cameraName,final int pipeline) {
       limelight = hardwareMap.get(Limelight3A.class, cameraName);
       limelight.pipelineSwitch(pipeline);
       limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public float getDistance() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()){
        final float ty = (float) result.getTy();
        return (ObjectHeight - camPose.translation.z) / MathFuncs.tan(Angle.degToRad(ty + camPose.rotation.y));
        }
        return 0;
    }

    public float getAngle() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()){
        return  Angle.degToRad((float) result.getTx());
        }
        return 0;
    }

    public float getHeading(){
        LLResult result = limelight.getLatestResult();
        if (result.isValid()){
        return Angle.wrapPlusMinusPI( OrbitPoseTracker.getHeading() + Angle.degToRad((float) result.getTx()));
        }
        return 0;
    }

    public Pose2D getTargetPose(){
        final float distance = getDistance();
        final float angle = getAngle(); // the direction of the target
        final float heading = getHeading();
        return new Pose2D(
                Vector.fromAngleAndRadius(angle,distance),
                heading
        );
    }
}
