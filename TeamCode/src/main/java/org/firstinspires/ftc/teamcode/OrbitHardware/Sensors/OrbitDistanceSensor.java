package org.firstinspires.ftc.teamcode.OrbitHardware.Sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor.PositionUnits;
import org.firstinspires.ftc.teamcode.robotData.Constants;

public class OrbitDistanceSensor {

  private  DistanceSensor distanceSensor;

  public OrbitDistanceSensor(final HardwareMap hardwareMap, final String name) {
    distanceSensor = hardwareMap.tryGet(DistanceSensor.class, name);
  }



  public float getDistance(final PositionUnits unit){
    final boolean distanceSensorAvailable = distanceSensor != null;
    switch (unit){
      case CM:
        return distanceSensorAvailable ? (float) distanceSensor.getDistance(DistanceUnit.CM) : Constants.INF;
      case M:
        return distanceSensorAvailable ? (float) distanceSensor.getDistance(DistanceUnit.METER) : Constants.INF;
      case INCH:
      default:
        return distanceSensorAvailable ? (float) distanceSensor.getDistance(DistanceUnit.INCH) : Constants.INF;
    }
  }

}
