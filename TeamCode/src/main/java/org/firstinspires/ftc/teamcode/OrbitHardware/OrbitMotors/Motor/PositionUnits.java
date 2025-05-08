package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitMotors.Motor;

public enum PositionUnits {
    CM,INCH,TICKS,M,RADS,DEGREES;

    public static PositionUnits fromVelUnits(final VelocityUnits velocityUnits){
        switch (velocityUnits){
            case TICKS_PER_SECOND:
                return TICKS;
            case DEG:
                return DEGREES;
            case RAD:
                return RADS;
            case CMS:
            case CMM:
                return CM;
            case MM:
            case MS:
                return M;
        }
        return INCH;
    }
}
