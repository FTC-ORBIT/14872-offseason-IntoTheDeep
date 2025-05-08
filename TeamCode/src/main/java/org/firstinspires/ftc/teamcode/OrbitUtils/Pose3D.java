package org.firstinspires.ftc.teamcode.OrbitUtils;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitCams.OrbitWebcam.TempVision.aprilTags.OrbitAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Pose3D {
    public Vector3D translation;
    public Vector3D rotation;
    private static final Pose3D zero = new Pose3D(Vector3D.zero(),Vector3D.zero());

    public Pose3D (final float x, final float y, final float z , final Vector3D rotation){
        this.translation = new Vector3D(x,y,z);
        this.rotation = rotation;
    }

    public Pose3D(final Vector3D translation, final Vector3D rotation){
        this.translation = translation;
        this.rotation = rotation;
    }

    public Pose3D(final Vector3D translation, final float roll, final float pitch, final float yaw) {
        this.translation = translation;
        this.rotation = new Vector3D(roll, pitch, yaw);
    }

    public Pose3D(final float x, final float y, final float z, final float roll, final float pitch, final float yaw) {
        this.translation = new Vector3D(x, y, z);
        this.rotation = new Vector3D(roll, pitch, yaw);
    }


    public Pose3D zero(){
    return zero;
    }

    public Pose3D add(final Pose3D other){
        final Vector3D newRotation = new Vector3D(Angle.wrapPlusMinusPI(this.rotation.x + other.rotation.x),
                Angle.wrapPlusMinusPI(this.rotation.y + other.rotation.y),
                Angle.wrapPlusMinusPI(this.rotation.z + other.rotation.z));
        return new Pose3D(translation.add(other.translation),
                newRotation);
    }

    public Pose3D add(final Pose2D other){
        final Vector3D newRotation = new Vector3D(translation.x,translation.y,Angle.wrapPlusMinusPI(this.rotation.z + other.rotation));
        return new Pose3D(this.translation.add(translation),
                newRotation);
    }

    public Pose3D subtract(final Pose3D other) {
        final Vector3D newRotation = new Vector3D(Angle.wrapPlusMinusPI(this.rotation.x - other.rotation.x),
                Angle.wrapPlusMinusPI(this.rotation.y - other.rotation.y),
                Angle.wrapPlusMinusPI(this.rotation.z - other.rotation.z));
        return new Pose3D(this.translation.subtract(other.translation),
                newRotation);
    }

    public Pose3D subtract(final Pose2D other){
        final Vector3D newRotation = new Vector3D(rotation.x, rotation.y, Angle.wrapPlusMinusPI(this.rotation.z - other.rotation));
        return new Pose3D(this.translation.subtract(other.translation), newRotation);
    }

    public Pose2D to2D(){
        return new Pose2D(translation.xy(),translation.getYaw());
    }

    public static Pose3D from2D(final Pose2D otherPos, final float z,final float pitch){
        Vector3D from2DTranslation = new Vector3D(otherPos.translation.x,otherPos.translation.y,z);
        return new Pose3D(from2DTranslation,0,pitch,otherPos.rotation);
    }

    public static Pose3D fromAprilTags(final AprilTagDetection detection){
      return OrbitAprilTag.getRobotPose3D(detection);
    }

    public static Pose3D fromRR(final Pose2d pose2d, final float z){
        Vector3D fromRR = new Vector3D((float) pose2d.position.x, (float) pose2d.position.y,z);
        return new Pose3D(fromRR,0,fromRR.getPitch(),fromRR.getYaw());
    }

    public static Pose2d toRR(final Pose3D pose3D ){
        return new Pose2d(pose3D.translation.x,pose3D.translation.y,pose3D.translation.getYaw());
    }

    public static Pose3D fromSDKPose3D(final org.firstinspires.ftc.robotcore.external.navigation.Pose3D pose3D){
        final Position position = pose3D.getPosition();
        final YawPitchRollAngles orientation = pose3D.getOrientation();
        return new Pose3D((float) position.x, (float) position.y, (float) position.z, (float) orientation.getRoll(), (float) orientation.getPitch(), (float) orientation.getYaw());
    }

    public boolean equals(final Pose3D otherPos){
        return translation.equals(otherPos.translation) && rotation.equals(otherPos.rotation);
    }

    public String toString(){
    return translation.toString() + "yaw:" + translation.getYaw() + "pitch:" + translation.getPitch();
    }

}
