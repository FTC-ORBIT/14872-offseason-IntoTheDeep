package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.ArmStates;

@TeleOp(name = "TuneArm")
public class TuneArm extends LinearOpMode {
    private final TelemetryPacket packet = new TelemetryPacket();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        //init
        Arm.init(hardwareMap, "armMotor", "armMotor2","armServo");

        waitForStart();

        while (!isStopRequested()){
            final ArmStates armState = gamepad1.y ? ArmStates.HIGH_BASKET : ArmStates.TRAVEL;

            Arm.operate(armState);
            packet.put("angle", Math.toDegrees(Arm.getAngle()));
            packet.put("wanted", Math.toDegrees(Arm.getWantedAngle()));
            packet.put("power",Arm.getPower());
//            packet.put("motor fault", Arm.armMotor.checkForMotorFault() ? 1 : 0);
//            packet.put("motor 2 fault", Arm.armMotor.checkForMotorFault() ? 1 : 0);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
