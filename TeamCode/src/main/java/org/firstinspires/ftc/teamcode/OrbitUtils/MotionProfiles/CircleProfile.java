package org.firstinspires.ftc.teamcode.OrbitUtils.MotionProfiles;

import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.robotData.Constants;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class CircleProfile extends MotionProfiles {


    public float velProfile(final float currVel, final float radius) {
        return calcVelByRadiusAndAccel(radius, calcAccel(radius, currVel));
    }

    public float calcAccel(final float radius, final float vel) {
        return MathFuncs.pow(vel, 2) / radius;
    }

    public float calcTangentialAccel(final float radius, final float angularVelocity) {
        return (angularVelocity * angularVelocity) * radius;
    }

    public float calcAngularVel(final float vel, final float radius) {
        return vel / radius;
    }

    public float calcLinearVelFromDistance(final float deltaDistance, final float radius) {
        return (deltaDistance / GlobalData.deltaTime) * radius;
    }

    public float calcLinearVelFromAngular(final float angularVelocity, final float radius) {
        return angularVelocity * radius;
    }

    public float calcVelByRadiusAndAccel(final float radius, final float accel) {
        return MathFuncs.sqrt(radius * accel);
    }


    public float calcTotalTimeByVel(final float radius, final float vel) {
        return (2 * Constants.PI * radius) / vel;
    }

    public float getAngleFromArcLength(final float arcLength, final float radius) {
        return MathFuncs.asin(arcLength / radius);
    }

    public float getArcLengthFromAngle(final float angle, final float radius) {
        return angle * radius;
    }

    public float calcTotalTimeByArcLength(final float ArcLength,final float linearVel){
    return ArcLength / linearVel;
    }

    public Vector getCircleCenter(final Vector start, final Vector end, final float radius, final boolean leftRoute) {

        final Vector midPoint = start.add(end).scale(0.5f);

        final float angle = MathFuncs.atan2(end.y - start.y, end.x - start.x);

        float theta = angle + (leftRoute ? Angle.halfPI : -Angle.halfPI);

        final float offsetX = radius * MathFuncs.sin(theta);
        final float offsetY = radius * MathFuncs.cos(theta);

        return new Vector(midPoint.x + offsetX, midPoint.y + offsetY);
    }


}
