package org.firstinspires.ftc.teamcode.testsOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
  import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

public final class testsOpModeManager {

    public static final String GROUP = "orbit tests";
    public static final boolean DISABLED = false;

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }
//    @OpModeRegistrar
//    public static void register(OpModeManager manager) {
//        if (DISABLED) return;
//        manager.register(metaForClass(DriveTrainOmniTest.class),new DriveTrainOmniTest());
//        manager.register(metaForClass(DriveTrainTankTest.class),new DriveTrainTankTest());
//        manager.register(metaForClass(SampleArmTest.class),new SampleArmTest());
//        manager.register(metaForClass(MotionProfilesAutoDrivesTests.class),new MotionProfilesAutoDrivesTests());
//        manager.register(metaForClass(AutoDriveTests.class),new AutoDriveTests());
//    }
}
