package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitGamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.OrbitHardware.HardwareColors;
import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

import java.util.ArrayList;
import java.util.List;

public class OrbitGamepad extends Gamepad{
    public OrbitGamepad() {
        super();
        for (int i = 0; i < ButtonsId.ids.size(); i++) {
            pressedButtons.add(false);
            rawButtons.add(false);
        }
    }

    private  final List<Boolean> pressedButtons = new ArrayList<>();
    private  final List<Boolean> rawButtons = new ArrayList<>();

    private static boolean prevA = false;
    private static boolean prevB = false;
    private static boolean prevX = false;
    private static boolean prevY = false;
    private static boolean prevDpadUp = false;
    private static boolean prevDpadDown = false;
    private static boolean prevDpadLeft = false;
    private static boolean prevDpadRight = false;
    private static boolean prevLeftBumper = false;
    private static boolean prevRightBumper = false;
    private static boolean prevLeftStickButton = false;
    private static boolean prevRightStickButton = false;
    private static boolean prevBack = false;
    private static boolean prevStart = false;
    private static boolean prevTouchPad = false;

    private List<Boolean> updatePressedButtons(){
    pressedButtons.set( ButtonsId.A, this.a && !prevA);
    pressedButtons.set( ButtonsId.B, this.b && !prevB);
    pressedButtons.set( ButtonsId.X, this.x && !prevX);
    pressedButtons.set( ButtonsId.Y, this.y && !prevY);
    pressedButtons.set( ButtonsId.DPAD_UP, this.dpad_up && !prevDpadUp);
    pressedButtons.set( ButtonsId.DPAD_DOWN, this.dpad_down && !prevDpadDown);
    pressedButtons.set( ButtonsId.DPAD_LEFT, this.dpad_left && !prevDpadLeft);
    pressedButtons.set( ButtonsId.DPAD_RIGHT, this.dpad_right && !prevDpadRight);
    pressedButtons.set( ButtonsId.LEFT_BUMPER, this.left_bumper && !prevLeftBumper);
    pressedButtons.set( ButtonsId.RIGHT_BUMPER, this.right_bumper && !prevRightBumper);
    pressedButtons.set( ButtonsId.LEFT_STICK_BUTTON, this.left_stick_button && !prevLeftStickButton);
    pressedButtons.set( ButtonsId.RIGHT_STICK_BUTTON, this.right_stick_button && !prevRightStickButton);
    pressedButtons.set( ButtonsId.BACK, this.back && !prevBack);
    pressedButtons.set( ButtonsId.START, this.start && !prevStart);
    pressedButtons.set( ButtonsId.TOUCHPAD, this.touchpad && !prevTouchPad);

    prevA = this.a;
    prevB = this.b;
    prevX = this.x;
    prevY = this.y;
    prevDpadUp = this.dpad_up;
    prevDpadDown = this.dpad_down;
    prevDpadLeft = this.dpad_left;
    prevDpadRight = this.dpad_right;
    prevLeftBumper = this.left_bumper;
    prevRightBumper = this.right_bumper;
    prevLeftStickButton = this.left_stick_button;
    prevRightStickButton = this.right_stick_button;
    prevBack = this.back;
    prevStart = this.start;
    prevTouchPad = this.touchpad;

    return pressedButtons;
    }

    private List<Boolean> updateRawButtons(){
    rawButtons.set( ButtonsId.A, this.a);
    rawButtons.set( ButtonsId.B, this.b);
    rawButtons.set( ButtonsId.X, this.x);
    rawButtons.set( ButtonsId.Y, this.y);
    rawButtons.set( ButtonsId.DPAD_UP, this.dpad_up);
    rawButtons.set( ButtonsId.DPAD_DOWN, this.dpad_down);
    rawButtons.set( ButtonsId.DPAD_LEFT, this.dpad_left);
    rawButtons.set( ButtonsId.DPAD_RIGHT, this.dpad_right);
    rawButtons.set( ButtonsId.LEFT_BUMPER, this.left_bumper);
    rawButtons.set( ButtonsId.RIGHT_BUMPER, this.right_bumper);
    rawButtons.set( ButtonsId.LEFT_STICK_BUTTON, this.left_stick_button);
    rawButtons.set( ButtonsId.RIGHT_STICK_BUTTON, this.right_stick_button);
    rawButtons.set( ButtonsId.BACK, this.back);
    rawButtons.set( ButtonsId.START, this.start);
    rawButtons.set( ButtonsId.TOUCHPAD, this.touchpad);

    return rawButtons;
    }

    public void updateButtons(){
    GlobalData.driverPressedButtons = updatePressedButtons();
    GlobalData.driverRawButtons = updateRawButtons();
    }

    public  List<Boolean> getButtonPressed(){ return pressedButtons;}
    public  List<Boolean> getButtonRaw(){ return rawButtons;}

    public boolean isButtonPressed(final int buttonId){
    return pressedButtons.get(buttonId);
    }


    public Vector leftJoyStick() {
        if (GlobalData.inAutonomous){
            return new Vector(0,0);
        }
        final float xAxis = MathFuncs.deadBand(this.left_stick_x,0.2f, 1);
        final float yAxis = MathFuncs.deadBand(this.left_stick_y,0.2f, 1);
        return new Vector(xAxis, yAxis);
    }

    public Vector rightJoyStick() {
        if (GlobalData.inAutonomous){
            return new Vector(0,0);
        }
        final float xAxis = MathFuncs.deadBand(this.right_stick_x,0.2f, 1);
        final float yAxis = MathFuncs.deadBand(this.right_stick_y,0.2f, 1);
        return new Vector(xAxis, yAxis);
    }

    public float rightTrigger() {
        return MathFuncs.deadBand(this.right_trigger, 0f, 1);
    }

    public float leftTrigger() {
        return MathFuncs.deadBand(this.left_trigger, 0f, 1);
    }

    public void rumble(){
        OrbitRumble.rumble(this,GlobalData.robotState);
    }

    public void setLedSColor(final HardwareColors ledsColor){
       this.setLedColor(ledsColor.r,ledsColor.g,ledsColor.b,0);
    }

    public Vector poseFromTouchPad(final Vector upperLeft,final Vector lowerRight){
        TouchPadPressToPose touchPadPressToPose = new TouchPadPressToPose(upperLeft,lowerRight);
        return touchPadPressToPose.getPoseFromPress(this);
    }

    public boolean getButtonState(int buttonId) {
         switch (buttonId) {
            case ButtonsId.A :  return this.a;
            case ButtonsId.B :  return this.b;
            case ButtonsId.X :  return this.x;
            case ButtonsId.Y :  return this.y;
            case ButtonsId.DPAD_UP :  return this.dpad_up;
            case ButtonsId.DPAD_DOWN :  return this.dpad_down;
            case ButtonsId.DPAD_LEFT :  return this.dpad_left;
            case ButtonsId.DPAD_RIGHT :  return this.dpad_right;
            case ButtonsId.LEFT_BUMPER :  return this.left_bumper;
            case ButtonsId.RIGHT_BUMPER :  return this.right_bumper;
            case ButtonsId.LEFT_STICK_BUTTON :  return this.left_stick_button;
            case ButtonsId.RIGHT_STICK_BUTTON :  return this.right_stick_button;
            case ButtonsId.BACK :  return this.back;
            case ButtonsId.START :  return this.start;
            case ButtonsId.TOUCHPAD :  return this.touchpad;
            default: return false;
        }
    }
}
