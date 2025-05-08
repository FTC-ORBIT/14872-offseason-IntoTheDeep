package org.firstinspires.ftc.teamcode.PoseTracker;



import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose3D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.roadRunner_1_0.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class OrbitPoseTracker {
    private static PoseMessage message = new PoseMessage(Pose2D.toRR(Pose2D.zero()));

    public static void setRRPos(final Pose2d endPos) {
        message = new PoseMessage(endPos);
    }

    public static void setPose(final Pose2D currentPos){
        message = new PoseMessage(Pose2D.toRR(currentPos));
    }


    public static float getX(){
        return (float) message.x;
    }

    public static float getY(){
        return (float) message.y;
    }

    public static float getHeading(){
        return (float) message.heading;
    }
    public static double getRRTimeStamp(){return message.timestamp;}

    public static Vector getPosition(){return new Vector((float) message.x, (float) message.y);}

    public static Pose2d getRobotPosRR(){
        return new Pose2d(message.x,message.y,message.heading);
    }

    public static Pose2D getRobotOrbitPose2D(){
        return new Pose2D(new Vector((float) message.x, (float) message.y), (float) message.heading);
    }

    public static Pose3D getRobotOrbitPos3D(){
        return Pose3D.from2D(getRobotOrbitPose2D(),0,0);
    }

    public static Pose2d getRobotPosRRFromTag(AprilTagDetection detection){
        return Pose2D.toRR(Pose2D.fromAprilTags(detection));
    }

    public static Pose2D predictPose(final float dt){
        return getRobotOrbitPose2D().add(new Pose2D(DrivetrainOmni.getVelocity_FieldCS().scale(dt), DrivetrainOmni.getAngularVelocity() * dt).add(new Pose2D(DrivetrainOmni.getAcceleration().scale(0.5f * MathFuncs.pow(dt,2)), DrivetrainOmni.getAngularAcceleration() * 0.5f * MathFuncs.pow(dt,2))));
    }

    public static float predictHeading(final float dt){
        return getHeading() + DrivetrainOmni.getAngularVelocity() * dt + 0.5f * DrivetrainOmni.getAngularAcceleration() * MathFuncs.pow(dt,2);
    }

    public static Vector predictPosition(final float dt){
        return getPosition().add(DrivetrainOmni.getVelocity_FieldCS().scale(dt)).add(DrivetrainOmni.getAcceleration().scale(0.5f * MathFuncs.pow(dt,2)));
    }

    public static Pose2D predictVelocity(final float dt){
        return new Pose2D(DrivetrainOmni.getVelocity_FieldCS().add(DrivetrainOmni.getAcceleration().scale(dt)), DrivetrainOmni.getAngularVelocity() + DrivetrainOmni.getAngularAcceleration() * dt);
    }

    public static Vector predictVelocityTranslation(final float dt){
        return DrivetrainOmni.getVelocity_FieldCS().add(DrivetrainOmni.getAcceleration().scale(dt));
    }

    public static float predictAngularVelocity(final float dt){
        return DrivetrainOmni.getAngularVelocity() + DrivetrainOmni.getAngularAcceleration() * dt;
    }


}
