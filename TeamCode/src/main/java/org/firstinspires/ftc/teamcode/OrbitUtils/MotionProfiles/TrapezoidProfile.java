package org.firstinspires.ftc.teamcode.OrbitUtils.MotionProfiles;

import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class TrapezoidProfile extends MotionProfiles{
    public float velProfile(final float currVel, final float distance, final float maxVel, final float accel) {
        final float accelVel = currVel + Math.signum(distance) * accel * GlobalData.deltaTime;
        final float decelVel = MathFuncs.sqrt(Math.abs(2 * accel * distance));
        return Math.min(Math.min(Math.abs(accelVel), decelVel), maxVel) * Math.signum(accelVel);
    }

    public float velProfile(final float currVel, final float distance, final float accel) {
        final float accelVel = currVel + Math.signum(distance) * accel * GlobalData.deltaTime;
        // TODO => check if linear approx is needed
        final float decelVel = MathFuncs.sqrt(Math.abs(2 * accel * distance));
        return Math.min(Math.min(Math.abs(accelVel), decelVel), maxVel) * Math.signum(accelVel);
    }


    public float calcTotalTime(final float distance,final float maxVel,final float accel){
        return calcAccelTimeByMaxVel(maxVel, accel) + (distance - 2 * calcAccelDist(maxVel, accel)) / maxVel;
    }



    public boolean atMaxVel(final float time,final float maxVel,final float accel){
        return time > calcAccelTimeByMaxVel(maxVel, accel) && time < calcTotalTime(0, maxVel, accel);
    }

    public boolean isDecelerating(final float time,final float maxVel,final float accel){
        return time > calcTotalTime(0, maxVel, accel);
    }
}
