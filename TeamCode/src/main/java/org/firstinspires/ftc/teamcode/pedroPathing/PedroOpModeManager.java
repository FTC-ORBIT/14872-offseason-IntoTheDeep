package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.automatic.ForwardVelocityTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.automatic.ForwardZeroPowerAccelerationTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.automatic.LateralZeroPowerAccelerationTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.automatic.StrafeVelocityTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.localization.ForwardTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.localization.LateralTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.localization.PedroLocalizationTest;
import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.localization.SensorGoBildaPinpointExample;
import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.localization.TurnTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.pid.CurvedBackAndForth;
import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.pid.StraightBackAndForth;

public class PedroOpModeManager {
    public static final String GROUP = "pedro tests";
    public static final boolean DISABLED = false;

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }
    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;
        manager.register(metaForClass(ForwardVelocityTuner.class),new ForwardVelocityTuner());
        manager.register(metaForClass(ForwardZeroPowerAccelerationTuner.class),new ForwardZeroPowerAccelerationTuner());
        manager.register(metaForClass(LateralZeroPowerAccelerationTuner.class),new LateralZeroPowerAccelerationTuner());
        manager.register(metaForClass(StrafeVelocityTuner.class),new StrafeVelocityTuner());
        manager.register(metaForClass(ForwardTuner.class),new ForwardTuner());
        manager.register(metaForClass(LateralTuner.class),new LateralTuner());
        manager.register(metaForClass(PedroLocalizationTest.class),new PedroLocalizationTest());
        manager.register(metaForClass(SensorGoBildaPinpointExample.class),new SensorGoBildaPinpointExample());
        manager.register(metaForClass(TurnTuner.class),new TurnTuner());
        manager.register(metaForClass(CurvedBackAndForth.class),new CurvedBackAndForth());
        manager.register(metaForClass(StraightBackAndForth.class),new StraightBackAndForth());
    }
}
