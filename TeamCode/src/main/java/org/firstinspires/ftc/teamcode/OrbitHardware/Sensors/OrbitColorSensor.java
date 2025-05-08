package org.firstinspires.ftc.teamcode.OrbitHardware.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OrbitHardware.HardwareColors;

public class OrbitColorSensor {

    private final ColorSensor colorSensor;

    public OrbitColorSensor(final HardwareMap hardwareMap, final String name) {
        colorSensor = hardwareMap.tryGet(ColorSensor.class, name);
    }


    public boolean hasGamePiece() {
        return matchGamePiece(HardwareColors.YELLOW) || matchGamePiece(HardwareColors.BLUE) || matchGamePiece(HardwareColors.RED);
    }

    public boolean matchGamePiece(final HardwareColors gamePieceColor) {
        final HardwareColors color = getColorFromSensorRead();
        return color == gamePieceColor;
    }

    public void printRGBA(Telemetry telemetry) {
        if (colorSensor != null) {
            telemetry.addData("red", colorSensor.red());
            telemetry.addData("green", colorSensor.green());
            telemetry.addData("blue", colorSensor.blue());
            telemetry.addData("alpha", colorSensor.alpha());
        }
    }


    public HardwareColors getColorFromSensorRead() {
        return colorSensor != null ? HardwareColors.getColorFromSensorRead(colorSensor.red(), colorSensor.green(), colorSensor.blue(), colorSensor.alpha()) : HardwareColors.ORBIT;
    }

    public boolean matchColors(final HardwareColors... colors) {
        final HardwareColors currentColor = getColorFromSensorRead();
        for (final HardwareColors c : colors) {
            if (currentColor == c) {
                return true;
            }
        }
        return false;
    }

}
