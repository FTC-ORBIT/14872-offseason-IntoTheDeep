package org.firstinspires.ftc.teamcode.robotData;

import org.firstinspires.ftc.teamcode.OrbitHardware.HardwareColors;

import java.util.ArrayList;

public enum GamePiece {
    SAMPLE(),SPECIMEN(),NONE();

    public final HardwareColors[] allowAbleColors = new HardwareColors[]{HardwareColors.YELLOW, HardwareColors.RED, HardwareColors.BLUE};

    public static GamePiece getAllowedGamePiece(){
    // TODO => implement this method by sensors used
        return null;
    }
}
