package org.firstinspires.ftc.teamcode.autonomous.RR.customActions;

import static org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager.setSubsystemToState;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.OrbitAutonomousGeneral;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;

public class AutoSubsystemAction implements Action {

    private final RobotState wantedAutoState;
    private final boolean waitForStop;
    private final ElapsedTime autoElapsedTime;
    private final float autoDelayTime;

    public AutoSubsystemAction(final RobotState state, final float time, final boolean isStopping) {
        this.wantedAutoState = state;
        this.autoDelayTime = time;
        this.waitForStop = isStopping;
        autoElapsedTime = new ElapsedTime();
        autoElapsedTime.reset();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        GlobalData.currentTime = (float) autoElapsedTime.seconds();

        telemetryPacket.put("time", autoElapsedTime.milliseconds());
        telemetryPacket.put("robotState", wantedAutoState);
        telemetryPacket.put("prev robot state", GlobalData.lastStateAuto);

        if (autoElapsedTime.seconds() >= autoDelayTime) {
            setSubsystemToState(wantedAutoState, false);
            GlobalData.lastStateAuto = wantedAutoState;
            boolean stoppedAutoMoving = OrbitAutonomousGeneral.roadRunnerDrive.checkStop() && autoElapsedTime.seconds() > 0.1;
            return !(stoppedAutoMoving || !waitForStop);
        } else {
            setSubsystemToState(GlobalData.lastStateAuto,false);
            return true;
        }
    }
}