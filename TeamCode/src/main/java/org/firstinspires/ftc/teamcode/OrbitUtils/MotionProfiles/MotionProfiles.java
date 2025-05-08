package org.firstinspires.ftc.teamcode.OrbitUtils.MotionProfiles;

import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.robotData.Constants;

public class MotionProfiles {


    public float maxVel;
    public float maxAcc;
    public float maxJerk;


    public void setProfileParams(final float maxVel, final float maxAcc,final float maxJerk) {
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
        this.maxJerk = maxJerk;
    }



    public void setMaxVel(final float maxVel) {
        this.maxVel = maxVel;
    }

    public void setMaxAcc(final float maxAcc) {
        this.maxAcc = maxAcc;
    }

    public void setMaxJerk(final float maxJerk) {
        this.maxJerk = maxJerk;
    }


    public float getMaxVel() {
        return maxVel;
    }

    public float getMaxAcc() {
        return maxAcc;
    }

    public float getMaxJerk() {
        return maxJerk;
    }


    public float calcAccelTime(final float distance,final float accel){
        return MathFuncs.sqrt(Math.abs(2 * distance / accel));
    }

    public float calcAccelTimeByMaxVel(final float maxVel,final float accel){
        return maxVel / accel;
    }

    public  float calcAccelDist(final float maxVel , final float accel){
        return  0.5f * accel * MathFuncs.pow(calcAccelTimeByMaxVel(maxVel, accel),2);
    }


    public float getDistanceTraveled(final float time,final float velocity,final float accel){
        return velocity * time + 0.5f * accel * MathFuncs.pow(time,2);
    }

    public float calcVelocity(final float time,final float maxVel,final float accel){
        if (time < calcAccelTimeByMaxVel(maxVel, accel)){
            return accel * time;
        }else if (time < calcAccelTimeByMaxVel(maxVel, accel) + (-2 * calcAccelDist(maxVel, accel)) / maxVel){
            return maxVel;
        }else{
            return 0;
        }
    }

}
