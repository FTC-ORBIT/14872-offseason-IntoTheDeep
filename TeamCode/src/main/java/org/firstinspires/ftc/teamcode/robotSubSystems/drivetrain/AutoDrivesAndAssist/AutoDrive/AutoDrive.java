package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.AutoDrive;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.OrbitUtils.MotionProfiles.CircleProfile;
import org.firstinspires.ftc.teamcode.OrbitUtils.MotionProfiles.PolygonProfile;
import org.firstinspires.ftc.teamcode.OrbitUtils.MotionProfiles.SCurveProfile;
import org.firstinspires.ftc.teamcode.OrbitUtils.MotionProfiles.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.OrbitUtils.OrbitLogger.CSVLogger;
import org.firstinspires.ftc.teamcode.OrbitUtils.OrbitLogger.CSVTitle;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.TimedPose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.robotData.Constants;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmniConstants;

import java.util.ArrayList;
import java.util.List;

public class AutoDrive extends OrbitAutoDrive {
    private Pose2D[] pose2DS = null;
    private List<TimedPose2D> timedPath = null;
    private final ElapsedTime elapsedTrajectoryTimer = new ElapsedTime();
    private float currentTimedTrajcetoryTime;
    private float prevTimedTrajectoryTime;
    private int poseIndex = 0;
    private float allowedDistanceError; // inches
    private float allowedHeadingError; // radians
    private float finalVel;
    private float maxVel;
    private Vector lastTargetTranslation = Vector.zero(); // for lerp calc
    private Vector startMotionTranslation = Vector.zero(); // for lerp calc
    private float lastLerpTargetHeading = 0; // for lerp calc
    private float startRotationMotionHeading = 0; // for lerp calc

    // motions profiles:
    private final TrapezoidProfile trapezoidPositionProfile;
    private final SCurveProfile sCurvePositionProfile;
    private final PolygonProfile polygonPositionProfile;
    private final CircleProfile circlePositionProfile;

    private final TrapezoidProfile trapezoidAngularProfile;
    private final SCurveProfile sCurveAngularProfile;
    private final PolygonProfile polygonAngularProfile;

    public AutoDrive(final float allowedDistanceError, final float allowedHeadingError, final float finalVel, final float maxVel) {
        this.allowedDistanceError = allowedDistanceError;
        this.allowedHeadingError = allowedHeadingError;
        this.finalVel = finalVel;
        this.maxVel = maxVel;
        this.trapezoidPositionProfile = new TrapezoidProfile();
        this.sCurvePositionProfile = new SCurveProfile();
        this.polygonPositionProfile = new PolygonProfile();
        this.trapezoidAngularProfile = new TrapezoidProfile();
        this.sCurveAngularProfile = new SCurveProfile();
        this.polygonAngularProfile = new PolygonProfile();

        sCurvePositionProfile.setProfileParams(
                maxVel != 0 ? maxVel : DrivetrainOmniConstants.maxVel,
                DrivetrainOmniConstants.maxAccel,
                DrivetrainOmniConstants.maxJerk
        );

        circlePositionProfile = new CircleProfile();


        sCurveAngularProfile.setProfileParams(
                DrivetrainOmniConstants.maxAngularVel,
                DrivetrainOmniConstants.maxAngularAccel,
                DrivetrainOmniConstants.maxAngularJerk
        );

        polygonPositionProfile.setProfileParams(
                maxVel != 0 ? maxVel : DrivetrainOmniConstants.maxVel,
                DrivetrainOmniConstants.maxAccel,
                DrivetrainOmniConstants.maxJerk
        );

        polygonAngularProfile.setProfileParams(
                DrivetrainOmniConstants.maxAngularVel,
                DrivetrainOmniConstants.maxAngularAccel,
                DrivetrainOmniConstants.maxAngularJerk
        );


        init();
    }

    public AutoDrive(final float allowedDistanceError, final float allowedHeadingError) {
        this(allowedDistanceError, allowedHeadingError, 0, 0);
    }

    public AutoDrive(final float allowedDistanceError, final float allowedHeadingError, final float maxVel) {
        this(allowedDistanceError, allowedHeadingError, 0, maxVel);
    }

    public AutoDrive(final float allowedDistanceError, final float allowedHeadingError, final float maxVel, final float maxAccel,final float maxJerk) {
        this(allowedDistanceError, allowedHeadingError, 0, maxVel);
        setMaxAccel(maxAccel);
        setMaxJerk(maxJerk);
    }

    public AutoDrive(final float allowedDistanceError, final float allowedHeadingError, final float maxVel, final float maxAccel,final float maxJerk,final float finalVel) {
        this(allowedDistanceError, allowedHeadingError, finalVel, maxVel);
        setMaxAccel(maxAccel);
        setMaxJerk(maxJerk);
    }

    public void setFinalVel(final float finalVel) {
        this.finalVel = finalVel;
    }

    public void setMaxVel(final float maxVel) {
        this.maxVel = maxVel;
        trapezoidPositionProfile.setMaxVel(maxVel);
        sCurvePositionProfile.setMaxVel(maxVel);
        polygonPositionProfile.setMaxVel(maxVel);
        circlePositionProfile.setMaxVel(maxVel);
    }

    public void setMaxAccel(final float maxAccel) {
        trapezoidPositionProfile.setMaxAcc(maxAccel);
        sCurvePositionProfile.setMaxAcc(maxAccel);
        polygonPositionProfile.setMaxAcc(maxAccel);
        circlePositionProfile.setMaxAcc(maxAccel);
    }


    public void setMaxJerk(final float maxJerk) {
        trapezoidPositionProfile.setMaxJerk(maxJerk);
        sCurvePositionProfile.setMaxJerk(maxJerk);
        polygonPositionProfile.setMaxJerk(maxJerk);
        circlePositionProfile.setMaxJerk(maxJerk);
    }

    public float getMaxVel() {
        return maxVel;
    }

    public float getMaxAccel() {
        return trapezoidPositionProfile.getMaxAcc();
    }

    public float getMaxJerk() {
        return trapezoidPositionProfile.getMaxJerk();
    }

    public void setAllowedDistanceError(final float allowedDistanceError) {
        this.allowedDistanceError = allowedDistanceError;
    }

    public void setAllowedHeadingError(final float allowedHeadingError) {
        this.allowedHeadingError = allowedHeadingError;
    }

    public void setPoseIndex(final int poseIndex) {
        this.poseIndex = poseIndex;
    }


    public void init() {
        poseIndex = 0;
        pose2DS = null;
        timedPath = null;
        startMotionTranslation = Vector.INF();
        startRotationMotionHeading = Constants.INF;
    }


    public Pose2D calcVel(final Pose2D targetPose) {

        if (arrivedToPose(targetPose)) {
            return Pose2D.zero();
        }

        final Pose2D deltaPos = targetPose.subtract(OrbitPoseTracker.getRobotOrbitPose2D());

        float positionVel = calcPositionVel(targetPose.translation);

        float angularVel = calcAngularVel(deltaPos.rotation);


        final Pose2D vel = new Pose2D(Vector.fromAngleAndRadius(deltaPos.translation.getAngle(), positionVel), angularVel);

        CSVLogger.set(CSVTitle.VEL_X, vel.translation.x);
        CSVLogger.set(CSVTitle.VEL_Y, vel.translation.y);
        CSVLogger.set(CSVTitle.VEL_HEADING, vel.rotation);

        return vel;
    }


    public float calcPositionVel(Vector targetTranslation) {
        final float distanceRemaining = targetTranslation.subtract(OrbitPoseTracker.getPosition()).norm();
        final float currentVel = DrivetrainOmni.getVelocity_FieldCS().norm();
        final float currentAcc = Math.max(DrivetrainOmniConstants.minAccel, DrivetrainOmni.getAcceleration().norm());
        final Vector currentPosition = OrbitPoseTracker.getPosition();


        float stoppingVel = MathFuncs.sqrt(2 * currentAcc * distanceRemaining);


        if (distanceRemaining < allowedDistanceError) {
            return 0;
        }


        final float velByDistance = currentVel +
                MathFuncs.sqrt(Math.abs(2 * currentAcc * distanceRemaining));

        final float velByTime = currentVel + currentAcc * GlobalData.deltaTime;

        final float trapezoidVel = trapezoidPositionProfile.velProfile(currentVel, distanceRemaining, maxVel != 0 ? maxVel : DrivetrainOmniConstants.maxVel, currentAcc);

        final float sCurveVel = sCurvePositionProfile.velProfile(GlobalData.deltaTime, distanceRemaining + currentPosition.norm(), currentPosition.norm());

        final float polygonVel = polygonPositionProfile.velProfile(GlobalData.deltaTime, distanceRemaining + currentPosition.norm(), currentPosition.norm());

        final float lerpedVel = calcPositionVelLerp(targetTranslation).norm();

        float positionVel = (float) MathFuncs.min(
                velByDistance,
                velByTime,
                trapezoidVel,
                sCurveVel,
                polygonVel,
                lerpedVel
        );

        positionVel = Math.min(positionVel, stoppingVel);


        positionVel = maxVel != 0 ? MathFuncs.limit(maxVel, positionVel) :
                MathFuncs.limit(DrivetrainOmniConstants.maxVel, positionVel);

        float scaleFactor = MathFuncs.range(0, 1, distanceRemaining / (distanceRemaining + 1)); // Smooth out the slowdown
        positionVel = (float) (finalVel + (positionVel - finalVel) * Math.pow(scaleFactor, 2));

        positionVel /= DrivetrainOmniConstants.maxVel;
        return positionVel;
    }

    public float calcAngularVel(float deltaRotation) {

        float stoppingAngularVel = MathFuncs.sqrt(2 * Math.max(DrivetrainOmniConstants.minAngularAccel, DrivetrainOmni.getAngularAcceleration()) * Math.abs(deltaRotation));

        if (Math.abs(deltaRotation) < allowedHeadingError) {
            return 0;
        }

        final float angularVelByDistance = DrivetrainOmni.getAngularVelocity() +
                MathFuncs.sqrt(Math.abs(2 * Math.max(DrivetrainOmniConstants.minAngularAccel, DrivetrainOmni.getAngularAcceleration()) * Math.abs(deltaRotation)));

        final float angularVelByTime = DrivetrainOmni.getAngularVelocity() + DrivetrainOmni.getAngularAcceleration() * GlobalData.deltaTime;

        final float trapezoidAngularVel = trapezoidAngularProfile.velProfile(DrivetrainOmni.getAngularVelocity(), Math.abs(deltaRotation), maxVel != 0 ? maxVel : DrivetrainOmniConstants.maxVel, DrivetrainOmni.getAngularAcceleration());

        final float sCurveAngularVel = sCurveAngularProfile.velProfile(GlobalData.deltaTime, Math.abs(deltaRotation), Math.abs(DrivetrainOmni.getAngularVelocity()));

        final float polygonAngularVel = polygonAngularProfile.velProfile(GlobalData.deltaTime, Math.abs(deltaRotation), Math.abs(DrivetrainOmni.getAngularVelocity()));

        final float lerpedAngularVel = calcAngularVelLerp(Angle.wrapPlusMinusPI(deltaRotation + OrbitPoseTracker.getHeading()));

        float angularVel = (float) MathFuncs.min(
                angularVelByDistance,
                angularVelByTime,
                trapezoidAngularVel,
                sCurveAngularVel,
                polygonAngularVel,
                lerpedAngularVel
        );


        angularVel = Math.min(angularVel, stoppingAngularVel);


        angularVel = MathFuncs.limit(DrivetrainOmniConstants.maxAngularVel, angularVel);

        return angularVel / DrivetrainOmniConstants.maxAngularVel;
    }


    public void driveToLerp(final Pose2D targetPose) {
        CSVLogger.set(CSVTitle.X_W, targetPose.translation.x);
        CSVLogger.set(CSVTitle.Y_W, targetPose.translation.y);
        CSVLogger.set(CSVTitle.HEADING_W, targetPose.rotation);
        moveRobot(calcLerpVel(targetPose));
    }

    public void rotateTo(final float targetHeading) {
        CSVLogger.set(CSVTitle.HEADING_W, targetHeading);
        moveRobot(calcVel(new Pose2D(OrbitPoseTracker.getPosition(), targetHeading)));
    }


    public Vector calcPositionVelLerp(final Vector targetTranslation) {
        if (!targetTranslation.equals(lastTargetTranslation)) {
            startMotionTranslation = OrbitPoseTracker.getPosition();
        }
        lastTargetTranslation = targetTranslation;

        Vector startMotionPoint = new Vector(startMotionTranslation.norm(), maxVel != 0 ? maxVel : DrivetrainOmniConstants.maxVel);
        Vector endMotionPoint = new Vector(targetTranslation.norm(), finalVel);
        float lerpedPositionVel = MathFuncs.twoPointsLinear(startMotionPoint, endMotionPoint, targetTranslation.subtract(OrbitPoseTracker.getPosition()).norm());

        lerpedPositionVel = Math.max(0, lerpedPositionVel);
        lerpedPositionVel = MathFuncs.limit(maxVel != 0 ? maxVel : DrivetrainOmniConstants.maxVel, lerpedPositionVel);

        return Vector.fromAngleAndRadius(targetTranslation.subtract(OrbitPoseTracker.getPosition()).getAngle(), lerpedPositionVel / DrivetrainOmniConstants.maxVel);
    }

    public float calcAngularVelLerp(final float targetHeading) {
        if (targetHeading != lastLerpTargetHeading) {
            startRotationMotionHeading = OrbitPoseTracker.getHeading();
        }
        lastLerpTargetHeading = targetHeading;

        final Vector startMotion = new Vector(startRotationMotionHeading, DrivetrainOmniConstants.maxAngularVel);
        final Vector endMotion = new Vector(targetHeading, 0);
        float lerpedAngularVel = MathFuncs.twoPointsLinear(startMotion, endMotion, Angle.wrapPlusMinusPI(targetHeading - OrbitPoseTracker.getHeading()));

        lerpedAngularVel = MathFuncs.limit(DrivetrainOmniConstants.maxAngularVel, lerpedAngularVel);

        return lerpedAngularVel / DrivetrainOmniConstants.maxAngularVel;
    }

    public Pose2D calcLerpVel(final Pose2D targetPose) {
        return new Pose2D(calcPositionVelLerp(targetPose.translation), calcAngularVelLerp(targetPose.rotation));
    }

    public Vector calcPositionSplineVel(final Vector targetTranslation, final boolean leftRoute) {

        if (!targetTranslation.equals(lastTargetTranslation)) {
            startMotionTranslation = OrbitPoseTracker.getPosition();
        }
        lastTargetTranslation = targetTranslation;


        final float diameter = targetTranslation.subtract(startMotionTranslation).norm();

        final float radius = diameter / 2;

        Vector circleCenter = circlePositionProfile.getCircleCenter(startMotionTranslation, targetTranslation, radius, leftRoute);


        float startAngle = startMotionTranslation.subtract(circleCenter).getAngle();

        float tangentDirection = startAngle + (leftRoute ? Angle.halfPI : -Angle.halfPI);


        float velByCircleProfile = circlePositionProfile.velProfile(DrivetrainOmni.getVelocity_FieldCS().norm(), radius);


        velByCircleProfile = MathFuncs.limit(maxVel != 0 ? maxVel : DrivetrainOmniConstants.maxVel, velByCircleProfile);


        return Vector.fromAngleAndRadius(tangentDirection, velByCircleProfile / DrivetrainOmniConstants.maxVel);
    }


    public Pose2D calcSplineVel(final Pose2D targetPose, final boolean leftRoute) {
        final Pose2D splineVel = new Pose2D(calcPositionSplineVel(targetPose.translation, leftRoute), calcAngularVel(targetPose.rotation));
        CSVLogger.set(CSVTitle.VEL_X, splineVel.translation.x);
        CSVLogger.set(CSVTitle.VEL_Y, splineVel.translation.y);
        CSVLogger.set(CSVTitle.VEL_HEADING, splineVel.rotation);
        return splineVel;
    }


    public void driveTo(final Pose2D targetPose) {
        CSVLogger.set(CSVTitle.X_W, targetPose.translation.x);
        CSVLogger.set(CSVTitle.Y_W, targetPose.translation.y);
        CSVLogger.set(CSVTitle.HEADING_W, targetPose.rotation);
        moveRobot(calcVel(targetPose));
    }


    public void splineTo(final Pose2D targetPose, final boolean leftRoute) {
        CSVLogger.set(CSVTitle.X_W, targetPose.translation.x);
        CSVLogger.set(CSVTitle.Y_W, targetPose.translation.y);
        CSVLogger.set(CSVTitle.HEADING_W, targetPose.rotation);
        moveRobot(calcSplineVel(targetPose, leftRoute));
    }

    public void splineTo(final Vector targetTranslation, final boolean leftRoute) {
        CSVLogger.set(CSVTitle.X_W, targetTranslation.x);
        CSVLogger.set(CSVTitle.Y_W, targetTranslation.y);
        CSVLogger.set(CSVTitle.HEADING_W, OrbitPoseTracker.getHeading());
        final Pose2D splineVel = new Pose2D(calcPositionSplineVel(targetTranslation, leftRoute), calcAngularVel(OrbitPoseTracker.getHeading()));
        moveRobot(splineVel);
    }

    public void splineTo(final Pose2D targetPose){
        final Pose2D deltaPos = targetPose.subtract(OrbitPoseTracker.getRobotOrbitPose2D());
        final Vector unit = Vector.unit(OrbitPoseTracker.getHeading());
        float cross = unit.x * deltaPos.getY() - unit.y * deltaPos.getX();
        boolean leftRoute = cross > 0; // we choose the shortest route to the target pose
        splineTo(targetPose, leftRoute);
    }

    public void splineTo(final Vector targetTranslation){
        final Vector deltaPos = targetTranslation.subtract(OrbitPoseTracker.getPosition());
        final Vector unit = Vector.unit(OrbitPoseTracker.getHeading());
        float cross = unit.x * deltaPos.y - unit.y * deltaPos.x;
        boolean leftRoute = cross > 0; // we choose the shortest route to the target pose
        splineTo(targetTranslation, leftRoute);
    }

    /// This method is used for trajectory following => more accurate than the timedTrajectory method
    public void trajectory(final Pose2D... pose2DS) {
        this.pose2DS = pose2DS;
        if (pose2DS == null || pose2DS.length == 0) {
            moveRobot(Pose2D.zero()); // Stop if no trajectory
            return;
        }

        Pose2D targetPose = pose2DS[poseIndex];
        if (arrivedToPose(targetPose)) {
            poseIndex++;
            if (poseIndex >= pose2DS.length) {
                moveRobot(Pose2D.zero());
                return;
            }
            targetPose = pose2DS[poseIndex];
        }
        driveTo(targetPose);
    }


    /// This method is used for timed trajectory following => faster than the trajectory method
    public void timedTrajectory(final Pose2D... poses) {
        if (poses == null || poses.length == 0) {
            moveRobot(Pose2D.zero());
            return;
        }

        if (timedPath == null) {
            timedPath = new ArrayList<>();
            elapsedTrajectoryTimer.reset(); // Reset the timer at the beginning
            currentTimedTrajcetoryTime = 0;
            prevTimedTrajectoryTime = 0;

            final float speed = maxVel != 0 ? maxVel : DrivetrainOmniConstants.maxVel;

            for (int i = 0; i < poses.length; i++) {
                Pose2D pose = poses[i];
                if (i > 0) {
                    float distance = pose.translation.subtract(poses[i - 1].translation).norm();
                    currentTimedTrajcetoryTime += distance / speed * 1000; // Convert to milliseconds
                }
                timedPath.add(new TimedPose2D(currentTimedTrajcetoryTime, pose));
            }
            poseIndex = 0;
        }

        if (poseIndex >= timedPath.size() - 1) {
            moveRobot(Pose2D.zero());
            return;
        }

        float elapsed = (float) (elapsedTrajectoryTimer.milliseconds() - prevTimedTrajectoryTime);

        TimedPose2D prev = timedPath.get(poseIndex);
        TimedPose2D next = timedPath.get(poseIndex + 1);

        float segmentTime = next.t - prev.t;
        float t = segmentTime == 0 ? 1 : elapsed / segmentTime;
        t = Math.max(0, Math.min(1, t));

        Pose2D targetPose = TimedPose2D.lerp(prev, next, t).pose;

        driveToLerp(targetPose);
    }


    /// this method is used for spline trajectory following => faster than the trajectory method
    public void splineTrajectory(final Pose2D... poses) {
        if (poses == null || poses.length == 0) {
            moveRobot(Pose2D.zero());
            return;
        }

        if (pose2DS == null || this.pose2DS != poses) {
            this.pose2DS = poses;
            this.poseIndex = 0;
        }

        if (poseIndex >= pose2DS.length) {
            moveRobot(Pose2D.zero());
            return;
        }

        Pose2D targetPose = pose2DS[poseIndex];
        if (arrivedToPose(targetPose)) {
            poseIndex++;
            if (poseIndex >= pose2DS.length) {
                moveRobot(Pose2D.zero());
                return;
            }
            targetPose = pose2DS[poseIndex];
        }


        splineTo(targetPose); // Use the splineTo method to follow the trajectory
    }

    /// this method is used for timed spline trajectory following => the fastest trajectory following method but not the most accurate
    public void timedSplineTrajectory(final Pose2D... poses) {
        if (poses == null || poses.length == 0) {
            moveRobot(Pose2D.zero());
            return;
        }

        if (timedPath == null) {
            timedPath = new ArrayList<>();
            elapsedTrajectoryTimer.reset(); // Reset the timer
            currentTimedTrajcetoryTime = 0;
            prevTimedTrajectoryTime = 0;

            final float speed = maxVel != 0 ? maxVel : DrivetrainOmniConstants.maxVel;

            for (int i = 0; i < poses.length; i++) {
                Pose2D pose = poses[i];
                if (i > 0) {
                    float distance = pose.translation.subtract(poses[i - 1].translation).norm();
                    currentTimedTrajcetoryTime += (distance / speed) * 1000; // ms
                }
                timedPath.add(new TimedPose2D(currentTimedTrajcetoryTime, pose));
            }
            poseIndex = 0;
        }

        if (poseIndex >= timedPath.size() - 1) {
            moveRobot(Pose2D.zero());
            return;
        }

        float elapsed = (float) (elapsedTrajectoryTimer.milliseconds() - prevTimedTrajectoryTime);

        TimedPose2D prev = timedPath.get(poseIndex);
        TimedPose2D next = timedPath.get(poseIndex + 1);

        float segmentTime = next.t - prev.t;
        float t = segmentTime == 0 ? 1 : elapsed / segmentTime;
        t = Math.max(0, Math.min(1, t));

        Pose2D targetPose = TimedPose2D.lerp(prev, next, t).pose;

        splineTo(targetPose);


        if (t >= 1.0f) {
            prevTimedTrajectoryTime = (float) elapsedTrajectoryTimer.milliseconds();
            poseIndex++;
        }
    }




    public boolean arrivedToPose(final Pose2D targetPos) {
        final Pose2D deltaPos = targetPos.subtract(OrbitPoseTracker.getRobotOrbitPose2D());
        return Math.abs(deltaPos.translation.norm()) <= allowedDistanceError && Math.abs(deltaPos.rotation) <= allowedHeadingError;
    }


    public boolean outFromTrajectory() {
        if (pose2DS == null || poseIndex >= pose2DS.length) {
            return true;
        }

        return timedPath == null || poseIndex >= timedPath.size();
    }

    public int getPoseIndex() {
        return poseIndex;
    }
}
