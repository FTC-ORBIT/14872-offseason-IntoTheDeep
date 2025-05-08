package org.firstinspires.ftc.teamcode.OrbitUtils.OrbitLogger;

import java.util.HashMap;
import java.util.Map;

import org.firstinspires.ftc.teamcode.robotData.Constants;
import org.firstinspires.ftc.teamcode.PoseTracker.OrbitPoseTracker;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotData.GamePiece;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmni;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DriveTrainOmni.DrivetrainOmniConstants;


public final class CSVLogger {

    private static final Map<CSVTitle, Float> values = new HashMap<CSVTitle, Float>();

    public static void set(CSVTitle title, float value) {
        values.put(title, value);
    }

    private static final CSVWriter csvWriter = new CSVWriter("Log", 7750, CSVTitle.getTitles(), true);

    public static void init() {
        defaultMap();
    }

    private static int getSpecialEvent() {
        int specialEvent = 0;
        if (GlobalData.gamePieceInRobot.equals(GamePiece.NONE) && GlobalData.hasGamePiece){
            specialEvent = 1;
        }
        if (!GlobalData.gamePieceInRobot.equals(GamePiece.NONE) && !GlobalData.hasGamePiece){
            specialEvent = 2;
        }
        if (GlobalData.voltage < DrivetrainOmniConstants.minVoltageForSuperSlowDrive){
            specialEvent = 3;
        }
        return specialEvent;
    }


    public static void update() {

        set(CSVTitle.TIME, GlobalData.currentTime);
        set(CSVTitle.VOLTAGE, GlobalData.voltage);
        set(CSVTitle.X_A, OrbitPoseTracker.getX());
        set(CSVTitle.Y_A, OrbitPoseTracker.getY());
        set(CSVTitle.HEADING_A,OrbitPoseTracker.getHeading());
        set(CSVTitle.VEL_X, DrivetrainOmni.getVelocity_FieldCS().x);
        set(CSVTitle.VEL_Y, DrivetrainOmni.getVelocity_FieldCS().y);
        set(CSVTitle.VEL_HEADING,DrivetrainOmni.getAngularVelocity());
        set(CSVTitle.ROBOT_STATE, GlobalData.robotState.ordinal());
        set(CSVTitle.JOYSTICK_X,DrivetrainOmni.getVelFromDriver().getX());
        set(CSVTitle.JOYSTICK_Y,DrivetrainOmni.getVelFromDriver().getY());
        set(CSVTitle.SPECIAL_EVENTS, getSpecialEvent());


        for (CSVTitle title : CSVTitle.values()) {
            Float value = values.get(title);
            csvWriter.addDataToLine(value);
        }
        csvWriter.endLine();
        defaultMap();
    }

    public static void defaultMap() {
        for (CSVTitle csvTitle : CSVTitle.values()) {
            values.put(csvTitle, -Constants.INF);
        }
    }

    public static void save() {
        csvWriter.saveFile();
    }
}

