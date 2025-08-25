package org.firstinspires.ftc.teamcode.robotData;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitGamepad.ButtonsId;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;
import org.firstinspires.ftc.teamcode.autonomous.OrbitPosMap;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;

import java.util.Collections;
import java.util.List;

@Config
public class GlobalData {
    public static RobotState robotState = RobotState.TRAVEL;
    public static RobotState lastStateAuto = RobotState.TRAVEL;
    public static boolean hasGamePiece = false;
    public static GamePiece gamePieceInRobot = GamePiece.NONE;
    public static boolean inAutonomous = false;
    public static float currentTime = 0;
    public static float lastTime = 0;
    public static float deltaTime = 0;
    public static float voltage = 0;
    public static boolean allianceColor = true; // true = blue, false = red
    public static boolean wasInAutonomous = false; // TODO => make this true in the end of all autos
    public static boolean inCompetition = false; // TODO => in the competition make this true
    public static boolean allowDriveByAprilTagsAssistAndAutoDrive = true;
    public static boolean allowDriveByObjectsAssistAndAutoDrive = true;
    public static boolean assistActive = false;
    public static boolean inAutoDrive = false;
    public static boolean firstCycleInTeleop = true;
    public static boolean usingDashBoardGampad = false;
    public static List<Boolean> driverPressedButtons = Collections.nCopies(ButtonsId.ids.size(), false);
    public static List<Boolean> driverRawButtons = Collections.nCopies(ButtonsId.ids.size(), false);
    public static OrbitPosMap posMap = new OrbitPosMap();
    public static float IntakeLength = 0f;
    public static float IntakeAngle = 0f;

    public static Vector rightStick = Vector.zero();
}


