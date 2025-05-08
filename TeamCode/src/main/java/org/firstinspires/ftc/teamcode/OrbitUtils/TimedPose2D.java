package org.firstinspires.ftc.teamcode.OrbitUtils;

public class TimedPose2D {
    public final float t;
    public final Pose2D pose;

    public TimedPose2D(float t, Pose2D pose) {
        this.t = t;
        this.pose = pose;
    }

    public static TimedPose2D lerp(TimedPose2D a, TimedPose2D b, float t) {

        t = Math.max(0, Math.min(1, t));


        float x = a.pose.translation.x + (b.pose.translation.x - a.pose.translation.x) * t;
        float y = a.pose.translation.y + (b.pose.translation.y - a.pose.translation.y) * t;
        Vector lerpedTranslation = new Vector(x, y);


        float deltaHeading = b.pose.rotation - a.pose.rotation;
        if (Math.abs(deltaHeading) > Math.PI) {
            deltaHeading -= Math.signum(deltaHeading) * 2 * Angle.pi;
        }
        float lerpedRotation = a.pose.rotation + deltaHeading * t;


        float lerpedTime = a.t + (b.t - a.t) * t;

        return new TimedPose2D(lerpedTime, new Pose2D(lerpedTranslation,lerpedRotation));
    }

}
