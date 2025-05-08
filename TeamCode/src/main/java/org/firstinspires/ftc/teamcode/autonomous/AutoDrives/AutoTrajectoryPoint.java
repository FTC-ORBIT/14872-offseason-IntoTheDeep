package org.firstinspires.ftc.teamcode.autonomous.AutoDrives;

import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;

public class AutoTrajectoryPoint {

    public Pose2D targetPose;
    private final Pose2D blueTargetPose;
    private final Pose2D redTargetPose;
    public RobotState wantedState;
    public boolean activateStateAction;

    public AutoTrajectoryPoint(final Pose2D blueTargetPose, final Pose2D redTargetPose, final RobotState wantedState, final boolean activateStateAction) {
        this.blueTargetPose = blueTargetPose;
        this.redTargetPose = redTargetPose;
        this.wantedState = wantedState;
        this.activateStateAction = activateStateAction;
    }

    public static void adjustPointByAllianceColor(final AutoTrajectoryPoint... autoTrajectoryPoint){
        for (AutoTrajectoryPoint point : autoTrajectoryPoint){
            if (GlobalData.allianceColor){
                point.targetPose = point.blueTargetPose;
            }else {
                point.targetPose = point.redTargetPose;
            }
        }
    }


    @Override
    public String toString() {
        return "AutoTrajectoryPoint :" +
                "targetPose =" + " " + targetPose +
                ", wantedState =" + " " + wantedState +
                ", activateStateAction =" + " " + activateStateAction
                ;
    }


}
