package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni;

import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.robotData.Constants;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class SmoothMovement {

    private static float previousVelocity = 0;

    public static Vector limitVelocityChange(final float maxAccel) {

        final Vector currentVelocity = DrivetrainOmni.getVelocity_FieldCS();
        float velocityChange = currentVelocity.norm() - previousVelocity;
        float maxChange = maxAccel * Constants.teleopCodeCycleTime;
        previousVelocity = currentVelocity.norm();
        float limitedVelocity = previousVelocity + Math.signum(velocityChange) * Math.min(Math.abs(velocityChange), maxChange);

        return currentVelocity.norm() == 0 ? currentVelocity : currentVelocity.scale(limitedVelocity / currentVelocity.norm());
    }


    public static Vector compensateTilt(final Vector targetVelocities, final float angle,final float height) {


        float adjustedX = targetVelocities.x * MathFuncs.cos(angle) - targetVelocities.y * MathFuncs.sin(angle);
        float adjustedY = targetVelocities.x * MathFuncs.sin(angle) + targetVelocities.y * MathFuncs.cos(angle);


        float heightInfluence = MathFuncs.log(height + 1); // there is 1 so that the log input is never 0, when the input is 1 we get 0

        float norm = (float) Math.sqrt(adjustedX * adjustedX + adjustedY * adjustedY);
        adjustedX /= norm * norm * heightInfluence;
        adjustedY /=  norm *  norm *  heightInfluence;

        return new Vector(adjustedX, adjustedY);
    }

    public static Vector slowDownFromEndFactorHeightAndAngle(final float endFactorAngle, final float endFactorHeight, final float maxAngle, final float maxHeight) {

        final Vector currentVelocities = DrivetrainOmni.getVelocity_FieldCS();

        float angleFactor = Math.max(0, (maxAngle - endFactorAngle) / maxAngle);
        float heightFactor = Math.max(0, (maxHeight - endFactorHeight) / maxHeight);

        final Vector angleAdjustment = currentVelocities.scale(angleFactor);
        final Vector heightAdjustment = currentVelocities.scale(heightFactor);


        final Vector adjustedVelocities = currentVelocities.subtract(angleAdjustment).subtract(heightAdjustment);

        return Vector.shortest(currentVelocities, adjustedVelocities);
    }

    public static Vector velByVoltage(final Vector targetVelocity) {

        if (GlobalData.voltage == 0){
            return targetVelocity; // its start at zero and in auto we use RR so we cant update it
        }

        if (GlobalData.voltage < DrivetrainOmniConstants.minVoltageForNormalDrive) {
            float scaleFactor = GlobalData.voltage / DrivetrainOmniConstants.minVoltageForNormalDrive;
            return targetVelocity.scale(scaleFactor);
        }

        return targetVelocity;
    }










}
