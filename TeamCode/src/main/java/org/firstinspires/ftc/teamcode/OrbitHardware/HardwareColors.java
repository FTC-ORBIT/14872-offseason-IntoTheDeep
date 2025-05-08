package org.firstinspires.ftc.teamcode.OrbitHardware;

import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;

public enum HardwareColors {
    ORBIT(0, 0, 255, 200, 10),
    RED(255, 0, 0, 255, 10),
    GREEN(0, 255, 0, 255, 10),
    BLUE(0, 0, 255, 255, 10),
    YELLOW(255, 255, 0, 255, 10),
    PURPLE(255, 0, 255, 255, 10),
    MAGENTA(255, 0, 255, 150, 10),
    CYAN(0, 255, 255, 255, 10),
    WHITE(255, 255, 255, 255, 10),
    BLACK(0, 0, 0, 0, 10);


    public final int r;
    public final int g;
    public final int b;
    public final int a;
    public final int colorRGBTolerance;

    HardwareColors(final int r, final int g, final int b, final int a, final int colorRGBTolerance) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.a = a;
        this.colorRGBTolerance = colorRGBTolerance;
    }


    public static HardwareColors getColorFromSensorRead(final int red, final int green, final int blue, final int alpha) {
        for (HardwareColors color : HardwareColors.values()) {
            if (MathFuncs.inTolerance(red, color.r, color.colorRGBTolerance) && MathFuncs.inTolerance(green, color.g, color.colorRGBTolerance) && MathFuncs.inTolerance(blue, color.b, color.colorRGBTolerance) && MathFuncs.inTolerance(alpha, color.a, 10)) {
                return color;
            }
        }
        return ORBIT; // default color
    }


    public static String toString(HardwareColors color) {
        switch (color) {
            case ORBIT:
                return "ORBIT";
            case RED:
                return "RED";
            case GREEN:
                return "GREEN";
            case BLUE:
                return "BLUE";
            case YELLOW:
                return "YELLOW";
            case PURPLE:
                return "PURPLE";
            case CYAN:
                return "CYAN";
            case WHITE:
                return "WHITE";
            case BLACK:
                return "BLACK";
            default:
                return "ORBIT";
        }
    }


    public static HardwareColors fromString(String color) {
        switch (color) {
            case "ORBIT":
                return ORBIT;
            case "RED":
                return RED;
            case "GREEN":
                return GREEN;
            case "BLUE":
                return BLUE;
            case "YELLOW":
                return YELLOW;
            case "PURPLE":
                return PURPLE;
            case "CYAN":
                return CYAN;
            case "WHITE":
                return WHITE;
            case "BLACK":
                return BLACK;
            default:
                return ORBIT;
        }
    }

    public int toArgb() {
        return android.graphics.Color.argb(a, r, g, b);
    }
}
