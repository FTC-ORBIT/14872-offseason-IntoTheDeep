package org.firstinspires.ftc.teamcode.OrbitUtils.OrbitLogger;


public enum CSVTitle {
    TIME("Time"),
    VOLTAGE("Voltage"),

    X_W("X_W"),
    Y_W("Y_W"),
    HEADING_W("Heading_W"),
    X_A("X_A"),
    Y_A("Y_A"),
    HEADING_A("Heading_A"),
    VEL_X("Vel_X"),
    VEL_Y("Vel_Y"),
    VEL_HEADING("Vel_Heading"),

    ROBOT_STATE("RobotState"),
    JOYSTICK_X("Joystick_x"),
    JOYSTICK_Y("Joystick_y"),

    AUTO_PEDRO_STATE("Auto_PedroState"),
    SPECIAL_EVENTS("SpecialEvents");

    private final String title;

    CSVTitle(final String title) {
        this.title = title;
    }

    public static String getTitles() {
        String titles = "";
        for (CSVTitle csvTitle : CSVTitle.values()) {
            if (CSVTitle.values()[CSVTitle.values().length - 1] != csvTitle)
                titles = titles + csvTitle.title + ", ";
            else
                titles = titles + csvTitle.title;
        }
        return titles;
    }


}