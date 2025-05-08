package org.firstinspires.ftc.teamcode.autonomous.Pedro.enable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;

import org.firstinspires.ftc.teamcode.autonomous.OrbitAutonomousGeneral;

import java.util.ArrayList;

@Autonomous(group = "enable")
public class ExampleForPedroAutoInOrbitWay extends OrbitAutonomousGeneral {




    @Override
    public void runOpMode() throws InterruptedException {
        initPedroPathing(Pose2D.zero());
        initRobot();


        ArrayList<PedroTrajectoryState> pedroTrajectoryStates = new ArrayList<>();
        pedroTrajectoryStates.add(PedroTrajectoryState.START_TO_BASKET);
        pedroTrajectoryStates.add(PedroTrajectoryState.BASKET_TO_SPIKE1);
        pedroTrajectoryStates.add(PedroTrajectoryState.SPIKE1_TO_BASKET);
        pedroTrajectoryStates.add(PedroTrajectoryState.BASKET_TO_SPIKE2);
        pedroTrajectoryStates.add(PedroTrajectoryState.SPIKE2_TO_BASKET);
        pedroTrajectoryStates.add(PedroTrajectoryState.BASKET_TO_SPIKE3);
        pedroTrajectoryStates.add(PedroTrajectoryState.SPIKE3_TO_BASKET);
        pedroTrajectoryStates.add(PedroTrajectoryState.BASKET_TO_PARK);
        pedroTrajectoryStates.add(PedroTrajectoryState.END_AUTO);

        waitForStart();

       runPedroTrajectory(pedroTrajectoryStates);

    }
}
