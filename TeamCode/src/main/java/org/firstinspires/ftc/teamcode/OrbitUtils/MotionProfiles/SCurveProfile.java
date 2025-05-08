package org.firstinspires.ftc.teamcode.OrbitUtils.MotionProfiles;

import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;

public class SCurveProfile extends MotionProfiles {

    public float jW = 0;
    public float aW = 0;
    public float vW = 0;

    private float prevAW = 0;
    private float prevVW = 0;


    public float velProfile(final float dt, final float target, final float current) {
        final float direction = Math.signum(target - current);
        final float remainingDistance = Math.abs(target - current);

        final float stopAcc = maxAcc;
        final float stopVel = (float) Math.sqrt(2 * stopAcc * remainingDistance);

        final float desiredVel = Math.min(stopVel, maxVel);
        final float desiredAcc = dt != 0 ? (desiredVel - prevVW) / dt : 0;


        float desiredJerk = dt != 0 ? (desiredAcc - prevAW) / dt : 0;
        desiredJerk = MathFuncs.limit(maxJerk, desiredJerk);


        jW = desiredJerk;
        aW = prevAW + jW * dt;
        aW = MathFuncs.limit(maxAcc, aW);

        vW = prevVW + aW * dt;
        vW = MathFuncs.limit(maxVel, vW);

        prevAW = aW;
        prevVW = vW;

        return direction * vW;
    }
}
