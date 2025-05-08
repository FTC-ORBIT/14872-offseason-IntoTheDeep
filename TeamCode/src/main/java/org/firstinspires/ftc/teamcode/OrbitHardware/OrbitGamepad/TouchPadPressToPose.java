package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitGamepad;


import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;

public class TouchPadPressToPose {
    private final Vector leftUpperPoint;
    private final Vector rightLowerPoint;
    private static boolean prevTouchPad = false;

    public TouchPadPressToPose(final Vector leftUpperPoint, final Vector rightLowerPoint) {
        this.leftUpperPoint = leftUpperPoint;
        this.rightLowerPoint = rightLowerPoint;
    }

    public Vector getPoseFromPress(final Gamepad gamepad) {
        if (gamepad.touchpad && !prevTouchPad) {
            return new Vector(
                    MathFuncs.deadBand(leftUpperPoint.x + rightLowerPoint.x / 2 +
                                    gamepad.touchpad_finger_1_x
                            , leftUpperPoint.x, rightLowerPoint.x),
                    MathFuncs.deadBand(leftUpperPoint.y + rightLowerPoint.y / 2 +
                                    gamepad.touchpad_finger_1_y
                            , rightLowerPoint.y, leftUpperPoint.y)
            );
        }
        prevTouchPad = gamepad.touchpad;
        return null;
    }

    public static Vector getPoseFromPress(final Vector leftUpperPoint, final Vector rightLowerPoint, final Gamepad gamepad) {
        if (gamepad.touchpad && !prevTouchPad) {
            return new Vector(
                    MathFuncs.deadBand(leftUpperPoint.x + rightLowerPoint.x / 2 +
                                    gamepad.touchpad_finger_1_x
                            , leftUpperPoint.x, rightLowerPoint.x),
                    MathFuncs.deadBand(leftUpperPoint.y + rightLowerPoint.y / 2 +
                                    gamepad.touchpad_finger_1_y
                            , rightLowerPoint.y, leftUpperPoint.y)
            );
        }
        prevTouchPad = gamepad.touchpad;
        return null;
    }
}
