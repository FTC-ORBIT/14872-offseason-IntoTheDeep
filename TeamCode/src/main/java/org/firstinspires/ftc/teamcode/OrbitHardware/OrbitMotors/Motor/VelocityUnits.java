package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor;


public enum VelocityUnits {
    INCH_PER_SECOND,MS,MM,CMS,CMM,RAD,DEG,TICKS_PER_SECOND;

    public static VelocityUnits fromPosUnits(final PositionUnits positionUnits){
        switch (positionUnits){
            case DEGREES:
                return DEG;
            case RADS:
                return RAD;
            case TICKS:
                return TICKS_PER_SECOND;
            case CM:
                return CMS;
            case M:
                return MS;
            case INCH:
                return INCH_PER_SECOND;

        }
        return INCH_PER_SECOND;
    }
}
