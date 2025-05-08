package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.Assists;

public interface OrbitAssistsInterFace {
    boolean shouldActivateAssist();
    void initProcessor();
    boolean seeTarget();
    void assistByTarget();
    void update();
}
