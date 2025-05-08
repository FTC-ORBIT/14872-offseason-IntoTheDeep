package org.firstinspires.ftc.teamcode.OrbitUtils.MotionProfiles;

import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;

public class PolygonProfile extends MotionProfiles {

    private float aW = 0;
    private float vW = 0;

    private float prevAW = 0;
    private float prevVW = 0;



    public float velProfile(final float dt, final float target, final float current) {
        final float direction = Math.signum(target - current);
        final float error = Math.abs(target - current);


        final float stopDist = (vW * vW) / (2 * maxAcc);

        boolean deceleratingPhase = stopDist >= error;

        float desiredAcc = deceleratingPhase ? -maxAcc : maxAcc;
        float desiredJerk = (desiredAcc - prevAW) / dt;

        desiredJerk = MathFuncs.limit(maxJerk, desiredJerk);

        float jW = desiredJerk;
        aW = prevAW + jW * dt;
        aW = MathFuncs.limit(maxAcc, aW);

        vW = prevVW + aW * dt;
        vW = MathFuncs.limit(maxVel, vW);


        prevAW = aW;
        prevVW = vW;

        return direction * vW;
    }

}
