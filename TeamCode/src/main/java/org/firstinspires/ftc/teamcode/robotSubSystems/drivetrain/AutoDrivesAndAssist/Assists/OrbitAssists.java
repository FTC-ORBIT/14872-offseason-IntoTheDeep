package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.Assists;

import com.qualcomm.robotcore.hardware.HardwareMap;
import android.graphics.Point;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.Assists.DriveByAprilTags.DriveByAprilTags;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.AutoDrive.OrbitAutoDrive;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class OrbitAssists implements OrbitAssistsInterFace {
    public static boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    public static double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    public static double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    public static double  turn            = 0;// Desired turning power/speed (-1 to +1)


    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public  static double SPEED_GAIN  =  0.03  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static double STRAFE_GAIN =  0.03 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public static double MAX_AUTO_SPEED = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_STRAFE= 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_TURN  = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)
    public static boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    public static VisionPortal visionPortal;

    public void  moveRobot(final Pose2D velocity){
        OrbitAutoDrive.moveRobot(velocity);
    }

    public void moveRobot(final Vector vel, final float omega){
        OrbitAutoDrive.moveRobot(vel,omega);
    }




}
