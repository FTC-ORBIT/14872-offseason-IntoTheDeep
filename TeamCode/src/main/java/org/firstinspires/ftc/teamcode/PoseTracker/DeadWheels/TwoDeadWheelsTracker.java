package org.firstinspires.ftc.teamcode.PoseTracker.DeadWheels;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.PositionUnits;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.VelocityUnits;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.OrbitMotor;
import org.firstinspires.ftc.teamcode.OrbitHardware.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitTracker;

public class TwoDeadWheelsTracker implements DeadWheelsTracker {
    private final OrbitMotor parallelDeadWheelMotor;
    private final OrbitMotor perpendicularDeadWheelMotor;
    private final PositionUnits positionUnit = PositionUnits.TICKS;
    private final VelocityUnits velocityUnit = VelocityUnits.TICKS_PER_SECOND;

    private float lastParPos = 0;
    private float lastPerpPos = 0;
    private float lastHeading = 0;

    private final float physicalUnit; // inches per tick

    public TwoDeadWheelsTracker(final HardwareMap hardwareMap, final OrbitMotor parallelDeadWheelMotor, final OrbitMotor perpendicularDeadWheelMotor, final float physicalUnit) {
        OrbitGyro.init(hardwareMap);
        this.parallelDeadWheelMotor = parallelDeadWheelMotor;
        this.perpendicularDeadWheelMotor = perpendicularDeadWheelMotor;
        this.physicalUnit = physicalUnit;

        reset(); // initialize state
    }

    public void reset() {
        this.lastParPos = getParPosition();
        this.lastPerpPos = getPerpPosition();
        this.lastHeading = getHeading();
    }

    public float getParVelocity() {
        return parallelDeadWheelMotor.getVelocity(velocityUnit);
    }


    public float getPerpVelocity() {
        return perpendicularDeadWheelMotor.getVelocity(velocityUnit);
    }


    public Vector getVelocity() {
        return new Vector(getParVelocity(), getPerpVelocity());
    }

    public float getParPosition() {
        return parallelDeadWheelMotor.getCurrentPosition(positionUnit);
    }

    public float getPerpPosition() {
        return perpendicularDeadWheelMotor.getCurrentPosition(positionUnit);
    }

    public float getHeading() {
        return (float) OrbitGyro.getAngle();
    }

    public Pose2D calcDeltaPose() {
        final float currentPar = getParPosition();
        final float currentPerp = getPerpPosition();
        final float currentHeading = getHeading();

        float deltaPar = (currentPar - lastParPos) * physicalUnit;
        float deltaPerp = (currentPerp - lastPerpPos) * physicalUnit;
        float deltaHeading = currentHeading - lastHeading;


        final float avgHeading = lastHeading + deltaHeading / 2f;
        final Vector delta = new Vector(deltaPar, deltaPerp).rotate(avgHeading);

        lastParPos = currentPar;
        lastPerpPos = currentPerp;
        lastHeading = currentHeading;


        return new Pose2D(delta, deltaHeading);
    }


    public Pose2D calcPose() {
       return OrbitPoseTracker.getRobotOrbitPose2D().add(calcDeltaPose());
    }

}
