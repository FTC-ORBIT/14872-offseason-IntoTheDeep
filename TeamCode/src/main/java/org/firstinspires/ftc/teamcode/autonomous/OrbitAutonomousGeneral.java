package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.robotSubSystems.RobotState.TRAVEL;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OrbitUtils.OrbitLogger.CSVLogger;
import org.firstinspires.ftc.teamcode.OrbitUtils.OrbitLogger.CSVTitle;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.ScoringAutomator;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;

import java.util.ArrayList;


public abstract class OrbitAutonomousGeneral extends LinearOpMode {
    public void initRobot() {// init only the subSystems and webcam. without Gyro or driveTrain
        ScoringAutomator.initAssists(hardwareMap, "webcam1", "webcam2");
        CSVLogger.init();
    }

    public static MecanumDrive roadRunnerDrive; // for RR
    public static Follower pedroFollower; // for PedroPathing

    public void initPedroPathing(final Pose2D startPos) {
        pedroFollower = new Follower(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        pedroFollower.setStartingPose(Pose2D.toPedro(startPos));
    }


    // create only the robot starting pos here, every otherPos you will create in GlobalPos
    public static Pose2D blueCloseToBasketStartPos = new Pose2D(0, 0, 0);
    public static Pose2D redCloseToBasketStartPos = new Pose2D(0, 0, 0);
    public static Pose2D redFarFromBasketStartPos = new Pose2D(0, 0, 0);
    public static Pose2D blueFarFromBasketStartPos = new Pose2D(0, 0, 0);

    public Pose2D switchStartPos(Gamepad gamepad1) {
        Pose2D pos = null;
        Pose2D lastPos = null;
        boolean posIsSet = false;
        String posName;
        String lastPosName = "none";
        while (pos == null || !posIsSet) {
            pos = gamepad1.a ? blueCloseToBasketStartPos // blue close to basket
                    : gamepad1.b ? blueFarFromBasketStartPos// blue far from basket
                    : gamepad1.x ? redCloseToBasketStartPos // red close to basket
                    : gamepad1.y ? redFarFromBasketStartPos // red far from basket
                    : lastPos;

            posName = gamepad1.a ? "blueCloseToBasketStartPos" // blue close to basket
                    : gamepad1.b ? "blueFarFromBasketStartPos"// blue far from basket
                    : gamepad1.x ? "redCloseToBasketStartPos" // red close to basket
                    : gamepad1.y ? "redFarFromBasketStartPos" // red far from basket
                    : lastPosName;

            if (pos != lastPos) lastPos = pos;

            if (!posName.equals(lastPosName)) lastPosName = posName;


            posIsSet = gamepad1.back;
            telemetry.addData("pos - ", posName);
            telemetry.addData("is set?", posIsSet);
            telemetry.update();
        }
        return pos;
    }

    public enum PedroTrajectoryState {

        START_TO_BASKET(new Path(new BezierLine(Pose2D.toPedro(OrbitAutonomousGeneral.blueCloseToBasketStartPos), Pose2D.toPedro(new Pose2D(0, 0, 0)))), TRAVEL, false),
        BASKET_TO_SPIKE1(new Path(new BezierCurve(Pose2D.toPedro(new Pose2D(0, 0, 0)), Pose2D.toPedro(new Pose2D(0, 0, 0)), Pose2D.toPedro(new Pose2D(0, 0, 0)))), TRAVEL, false),
        SPIKE1_TO_BASKET(null, TRAVEL, false),
        BASKET_TO_SPIKE2(null, TRAVEL, false),
        SPIKE2_TO_BASKET(null, TRAVEL, false),
        BASKET_TO_SPIKE3(null, TRAVEL, false),
        SPIKE3_TO_BASKET(null, TRAVEL, false),
        BASKET_TO_PARK(null, TRAVEL, false),
        END_AUTO(null, TRAVEL, false);

        public final Path path;
        public final RobotState robotState;
        public final boolean activateStateAction;

        PedroTrajectoryState(Path path, RobotState robotState, boolean activateStateAction) {
            this.path = path;
            this.robotState = robotState;
            this.activateStateAction = activateStateAction;
        }
    }

    private PedroTrajectoryState currentState = PedroTrajectoryState.START_TO_BASKET;
    private int trajectoryStateIndex = 0;

    public void runPedroTrajectory(final ArrayList<PedroTrajectoryState> pedroTrajectoryStates) {
        GlobalData.inAutonomous = true;

        ElapsedTime robotTime = new ElapsedTime();
        robotTime.reset();

        while (!isStopRequested() && !currentState.equals(PedroTrajectoryState.END_AUTO)) {
            GlobalData.currentTime = (float) robotTime.seconds();

            if (currentState.path != null) {
                pedroFollower.followPath(currentState.path);
            }

            if (!pedroFollower.isBusy() && SubSystemManager.systemsReady()) {
                trajectoryStateIndex++;
                if (trajectoryStateIndex < pedroTrajectoryStates.size()) {
                    currentState = pedroTrajectoryStates.get(trajectoryStateIndex);
                } else {
                    currentState = PedroTrajectoryState.END_AUTO;
                }
            }
            SubSystemManager.setSubsystemToState(currentState.robotState, false);

            telemetry.addData("Current State", currentState.name());
            telemetry.addData("robot Pose", pedroFollower.getPose());
            telemetry.update();

            CSVLogger.set(CSVTitle.AUTO_PEDRO_STATE, currentState.ordinal());
            CSVLogger.update();
        }
        currentState = PedroTrajectoryState.START_TO_BASKET;
        trajectoryStateIndex = 0;
        GlobalData.inAutonomous = false;
        GlobalData.wasInAutonomous = true;
        CSVLogger.save();
    }
}
