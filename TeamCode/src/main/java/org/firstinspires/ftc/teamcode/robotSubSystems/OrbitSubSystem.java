package org.firstinspires.ftc.teamcode.robotSubSystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface OrbitSubSystem {
    static void init(final HardwareMap hardwareMap, String... names) {

    }

    static void tune(final Telemetry telemetry){
      telemetry.addLine("not implemented");
      telemetry.update();
    }

    static void setSystemPower(final float power) {
        throw new UnsupportedOperationException("setSystemPower not implemented");
    }

    static float predict(final float dt) {
        return 0; // need to be => x = v * dt
    }

    static float getArbitraryF() {
        return 0;
    }

    static boolean haveFault(){
      return false;
    }
}
