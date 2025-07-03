package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.PositionUnits;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.OrbitMotor;
import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.OrbitUtils.OrbitLogger.CSVLogger;
import org.firstinspires.ftc.teamcode.OrbitUtils.OrbitLogger.CSVTitle;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.OrbitHardware.Sensors.OrbitGyro;
import org.firstinspires.ftc.teamcode.PoseTracker.DeadWheels.TwoDeadWheelsTracker;
import org.firstinspires.ftc.teamcode.PoseTracker.DriveTrain.OmniDriveEncodersTracker;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.roadRunner_1_0.Localizer;
import org.firstinspires.ftc.teamcode.roadRunner_1_0.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class    DrivetrainOmni {

    public static final OrbitMotor[] motors = new OrbitMotor[4];
    public static float driveFactor = DrivetrainOmniConstants.power;
    public static Vector lastPosition = Vector.zero();
    // equal to the last Autonomous position?
    public static Vector lastVelocity = GlobalData.inAutonomous ? getVelocity_FieldCS() : Vector.zero();
    public static float lastAngle = 0;
    private static float omegaWanted = 0f;
    private static Vector velocity_RobotCS_W = Vector.zero();
    private static float prevAngularVelocity = 0;
    private static OmniDriveEncodersTracker omniDriveEncodersTracker;
    private static TwoDeadWheelsTracker twoDeadWheelsTracker;
    private static TwoDeadWheelLocalizer twoDeadWheelLocalizer;


    public static void init(HardwareMap hardwareMap) {
        motors[0] = OrbitMotor.setFactorySettings(hardwareMap, "lf", DcMotorSimple.Direction.REVERSE);
        motors[1] = OrbitMotor.setFactorySettings(hardwareMap, "rf", DcMotorSimple.Direction.FORWARD);
        motors[2] = OrbitMotor.setFactorySettings(hardwareMap, "lb", DcMotorSimple.Direction.REVERSE);
        motors[3] = OrbitMotor.setFactorySettings(hardwareMap, "rb", DcMotorSimple.Direction.FORWARD);

//        omniDriveEncodersTracker = new OmniDriveEncodersTracker(hardwareMap,motors[0], motors[1], motors[2], motors[3],DrivetrainOmniConstants.inPerTick);
        twoDeadWheelsTracker = new TwoDeadWheelsTracker(hardwareMap,motors[0],motors[1],DrivetrainOmniConstants.inPerTick);
        twoDeadWheelLocalizer = new TwoDeadWheelLocalizer(hardwareMap, OrbitGyro.imu ,DrivetrainOmniConstants.inPerTick);


        if (GlobalData.wasInAutonomous) {
            lastPosition = OrbitPoseTracker.getPosition();
            lastVelocity = getVelocity_FieldCS();
            lastAngle = OrbitPoseTracker.getHeading();
        }
    }

    public static void operate(final Vector velocity_W, float omega) {
        final float robotAngle = (float) Math.toRadians(OrbitGyro.getAngle());
        velocity_RobotCS_W = velocity_W.rotate(-robotAngle);

        omegaWanted = omega;


        if (velocity_RobotCS_W.norm() <= 0.01 && Math.abs(omegaWanted) == 0) {
            stop();
        } else if (!GlobalData.assistActive && !GlobalData.inAutoDrive) {
            drive(velocity_RobotCS_W, omegaWanted);
        }
    }
    // did field centric


    public static Vector getVelocity_FieldCS() {
        Vector position = OrbitPoseTracker.getPosition();
        Vector deltaPosition = position.subtract(lastPosition);

        final Vector velocity = deltaPosition.scale(1 / GlobalData.deltaTime);

        lastPosition = position;
        return velocity;
    }

    public static Vector getAcceleration() {
        Vector currentVelocity = getVelocity_FieldCS();

        Vector deltaVelocity = currentVelocity.subtract(lastVelocity);
        Vector acceleration = deltaVelocity.scale(1 / GlobalData.deltaTime);

        lastVelocity = currentVelocity;
        return acceleration;
    }

    public static float getAngularVelocity() {
        final float angle = OrbitPoseTracker.getHeading();
        final float angularVel = Angle.wrapPlusMinusPI(angle - lastAngle ) / GlobalData.deltaTime;
        lastAngle = angle;
        return angularVel;
    }

    public static float getAngularAcceleration() {
        final float angularVel = getAngularVelocity();
        final float angularAccel = (angularVel - prevAngularVelocity) / GlobalData.deltaTime;
        prevAngularVelocity = angularVel;
        return angularAccel;
    }

    public static Pose2D getVelocity(){
        return new Pose2D(getVelocity_FieldCS(),getAngularVelocity());
    }


    public static void stop() {
        for (OrbitMotor motor : motors) {
            motor.setPower(0);
        }
    }

    public static void drive(Vector drive, final float r) {
        // add the tilt compensation from SmoothMovement if needed

      if (!GlobalData.inAutonomous) {drive = SmoothMovement.velByVoltage(drive);}

        CSVLogger.set(CSVTitle.VEL_X, drive.x);
        CSVLogger.set(CSVTitle.VEL_Y, drive.y);
        CSVLogger.set(CSVTitle.VEL_HEADING, r);

        final double lfPower = drive.x + drive.y + r;
        final double rfPower = drive.x - drive.y - r;
        final double lbPower = drive.x - drive.y + r;
        final double rbPower = drive.x + drive.y - r;
        double highestPower = 1;
        final double max = Math.max(Math.abs(lfPower),
                Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower))));

        if (max > 1) highestPower = max;

        final float ks = GlobalData.inAutonomous || GlobalData.inAutoDrive ? DrivetrainOmniConstants.ks : 0;

        motors[0].setPower((float) (driveFactor * (lfPower / highestPower) + ks * Math.signum(lfPower)));
        motors[1].setPower((float) (driveFactor * (rfPower / highestPower) + ks * Math.signum(rfPower)));
        motors[2].setPower((float) (driveFactor * (lbPower / highestPower) + ks * Math.signum(lbPower)));
        motors[3].setPower((float) (driveFactor * (rbPower / highestPower) + ks * Math.signum(rbPower)));

    }

    public static void turnByGivenPower(final float power) {
        motors[0].setPower(power);
        motors[1].setPower(-power);
        motors[2].setPower(power);
        motors[3].setPower(-power);
    }

    public static Vector getPositionVelFromDriver() {
        return velocity_RobotCS_W;
    }

    public static float getOmega() {
        return omegaWanted;
    }

    public static Pose2D getVelFromDriver(){
        return new Pose2D(velocity_RobotCS_W,omegaWanted);
    }

    public static Pose2D getPose() {
        final Pose2D driveEncodersPose = omniDriveEncodersTracker.calcPose();
        final Pose2D deadWheelsPose = twoDeadWheelsTracker.calcPose();

        final Twist2dDual<Time> twist = twoDeadWheelLocalizer.update();
        final Pose2d current = Pose2D.toRR(OrbitPoseTracker.getRobotOrbitPose2D());
        final Pose2D RRDeadWheels = Pose2D.fromRR(current.plus(twist.value()));

       return MathFuncs.average(driveEncodersPose, deadWheelsPose, RRDeadWheels);
    }

    public static void testEncoder(Telemetry telemetry) {
        telemetry.addData("lb", motors[2].getCurrentPosition(PositionUnits.TICKS));
        telemetry.addData("lf", motors[0].getCurrentPosition(PositionUnits.TICKS));
        telemetry.addData("rb-2", motors[3].getCurrentPosition(PositionUnits.TICKS));
        telemetry.addData("rf-3", motors[1].getCurrentPosition(PositionUnits.TICKS));
    }

    public static void testMotors(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.dpad_down) {
            motors[0].setPower(0.2f);
        } else if (gamepad.dpad_left) {
            motors[1].setPower(0.2f);
        } else if (gamepad.dpad_up) {
            motors[2].setPower(0.2f);
        } else if (gamepad.dpad_right) {
            motors[3].setPower(0.2f);
        }
        telemetry.addData("lf", motors[0].getCurrentPosition(PositionUnits.TICKS));
        telemetry.addData("rf", motors[1].getCurrentPosition(PositionUnits.TICKS));
        telemetry.addData("lb", motors[2].getCurrentPosition(PositionUnits.TICKS));
        telemetry.addData("rb", motors[3].getCurrentPosition(PositionUnits.TICKS));
    }

    public static void testDeadWheels(Telemetry telemetry) {
        telemetry.addData("par", motors[0].getCurrentPosition(PositionUnits.TICKS));
        telemetry.addData("prep", motors[1].getCurrentPosition(PositionUnits.TICKS));
    }
}
//dani yalechan!
// yoel yalechan!