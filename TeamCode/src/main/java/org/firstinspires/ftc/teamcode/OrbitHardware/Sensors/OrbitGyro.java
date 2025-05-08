
package org.firstinspires.ftc.teamcode.OrbitHardware.Sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.OrbitUtils.PID;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;

@Config
public class OrbitGyro {
    public static IMU imu;
    public static double lastAngle = 0;
    static double currentAngle = 0;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    private static final double iZone = 0;
    private static final float degForTolerance = 10;
    private static float wantedAngle;
    private static final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    private  static final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static void init(HardwareMap hardwareMap){
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection
        ));
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
    }
    public static void resetGyro (){
        lastAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public static void resetGyroStartTeleop (float angle){
        lastAngle = -angle;
    }

    public static double getAngle() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngle;
    }

    public static double getDAngle (){
        double lastAngleT = currentAngle;
        currentAngle = getAngle();
        return currentAngle - lastAngleT;
    }



    public static boolean inPose(){
        return MathFuncs.inTolerance((float) getAngle(), wantedAngle , degForTolerance);
    }

    public static void setWanted(final float angleError){
        wantedAngle = (float) ((Math.abs(angleError) + Math.abs(getAngle())) * Math.signum(getDAngle()) ) ;
    }
    public static float getAngularVelocity(){
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }



    public static boolean inPoseWithAngularVelocity(){
        return MathFuncs.inTolerance(getAngularVelocity(), wantedAngle , degForTolerance);
    }

    public static void setWantedWithAngularVelocity(final float angleError){
        wantedAngle = (float) ((Math.abs(angleError) + Math.abs(getAngularVelocity())) * Math.signum(getDAngle()) ) ;
    }
}
