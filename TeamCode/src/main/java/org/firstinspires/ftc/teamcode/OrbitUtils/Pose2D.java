package org.firstinspires.ftc.teamcode.OrbitUtils;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitCams.OrbitWebcam.TempVision.aprilTags.OrbitAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Pose2D {
    public Vector translation = Vector.zero();
    public float rotation = 0;
    private static final Pose2D zero = new Pose2D(Vector.zero(),0);

    public static Pose2D zero() {
        return zero;
    }

    public Pose2D(final float x, final float y, final float heading){
        translation.x = x;
        translation.y = y;
        rotation = heading;
    }
    public Pose2D(final Vector translation, final float rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    public Pose2D subtract(final Pose2D other) {
        return new Pose2D(translation.subtract(other.translation),
                Angle.wrapPlusMinusPI(rotation - other.rotation));
    }
    public Pose2D add(final Pose2D other){
        return new Pose2D(translation.add(other.translation), Angle.wrapPlusMinusPI(rotation + other.rotation));
    }

    public static Pose2D fromAprilTags(final AprilTagDetection detection){
        return OrbitAprilTag.getRobotPose(detection);
    }

    public static Pose2D fromRR(final Pose2d pose2d){
        return new Pose2D((float) pose2d.position.x, (float) pose2d.position.y, (float) pose2d.heading.toDouble());
    }

    public static Pose2d toRR(final Pose2D pose2D){
        return new Pose2d(pose2D.translation.x,pose2D.translation.y,pose2D.rotation);
    }

    public static Pose2D[] fromRR(final Pose2d... poses){
        Pose2D[] pose2Ds = new Pose2D[poses.length];
        for (int i = 0; i < poses.length; i++) {
            pose2Ds[i] = fromRR(poses[i]);
        }
        return pose2Ds;
    }

    public static Pose2d[] toRR(final Pose2D... pose2Ds){
        Pose2d[] poses = new Pose2d[pose2Ds.length];
        for (int i = 0; i < pose2Ds.length; i++) {
            poses[i] = toRR(pose2Ds[i]);
        }
        return poses;
    }

    public static Pose2D fromPedro(final Pose pose){
        return new Pose2D((float) pose.getX(), (float) pose.getY(), (float) pose.getHeading());
    }

    public static Pose toPedro(final Pose2D pose2D){
        return new Pose(pose2D.translation.x,pose2D.translation.y,pose2D.rotation);
    }

    public static Pose2D[] fromPedro(final Pose... poses){
        Pose2D[] pose2Ds = new Pose2D[poses.length];
        for (int i = 0; i < poses.length; i++) {
            pose2Ds[i] = fromPedro(poses[i]);
        }
        return pose2Ds;
    }

    public static Pose[] toPedro(final Pose2D... pose2Ds){
        Pose[] poses = new Pose[pose2Ds.length];
        for (int i = 0; i < pose2Ds.length; i++) {
            poses[i] = toPedro(pose2Ds[i]);
        }
        return poses;
    }

    public static Pose2D from3D(final Pose3D pose3D){
        return new Pose2D(pose3D.translation.xy(),pose3D.rotation.z);
    }

    public float getX(){
        return translation.x;
    }
    public float getY(){
        return translation.y;
    }
    public float getHeading(){
        return rotation;
    }

    public boolean equals(Pose2D pose2D){
        return translation.equals(pose2D.translation) && rotation == pose2D.rotation;
    }

    public Pose2D scale(final float scalingFactor){
        return new Pose2D(this.translation.scale(scalingFactor),Angle.wrapPlusMinusPI(this.rotation * scalingFactor));
    }



    @Override
    public String toString() {
        return translation.toString() + " angle (deg): " + Angle.radToDeg(rotation);
    }
}
