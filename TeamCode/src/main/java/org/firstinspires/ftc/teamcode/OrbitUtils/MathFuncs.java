package org.firstinspires.ftc.teamcode.OrbitUtils;

import org.firstinspires.ftc.teamcode.robotData.Constants;

public final class MathFuncs {
    public static Double max(double... doubleList) {
        double biggest =0;
        for(double i : doubleList) {
            if (i > biggest) {
                biggest = i;
            }
        }
        return biggest;
    }
    public static Double max(Object... objectList) {
        double biggest =0;
        double[] doubleList = new double[objectList.length];
        for(int i=0 ; i < objectList.length ; i++ ) {
            doubleList[i] = (double) objectList[i];
        }
        for(double i : doubleList) {
            if (i > biggest) {
                biggest = i;
            }
        }
        return biggest;
    }

    public static double min(double... DoubleList) {
        double smallest =0;
        for(double i : DoubleList) {
            if (i < smallest) {
                smallest = i;
            }
        }
        return smallest;
    }

    public static float deadBand(final float val, final float xAtZero, final float xAtOne) {
        final float returnVal = (Math.abs(val) - xAtZero) / (xAtOne - xAtZero);
        return range(0, 1, returnVal) * Math.signum(val);
    }

    public static float pointAndSlopeLinear(final Vector point, final float slope, final float value) {
        return slope * (value - point.x) + point.y;
    }

    public static float twoPointsLinear(final Vector pointA, final Vector pointB, final float value) {
        return (pointA.x == pointB.x) ? Constants.INF
                : pointAndSlopeLinear(pointA, (pointA.y - pointB.y) / (pointA.x - pointB.x), value);
    }

    public static float deadBand(final Vector point, final float slope, final float minVal, final float maxVal,
                                 final float value) {
        return range(minVal, maxVal, pointAndSlopeLinear(point, slope, value));
    }

    public static float deadBand(final Vector a, final Vector b, final float value) {
        final float maxValue = Math.max(a.y, b.y);
        final float minValue = Math.min(a.y, b.y);
        return range(minValue, maxValue, twoPointsLinear(a, b, value));
    }

    public static float range(final float lowerBound, final float upperBound, final float value) {
        return Math.min(Math.max(lowerBound, value), upperBound);
    }

    public static float limit(final float bound, final float value) {
        return range(-Math.abs(bound), Math.abs(bound), value);
    }

    public static boolean inRange(final float value, final float lowerBound, final float upperBound) {
        return value <= upperBound && value >= lowerBound;
    }

    public static boolean inTolerance(final float value, final float wantedValue, final float tolerance) {
        return inRange(value, wantedValue - tolerance, wantedValue + tolerance);
    }

    public static float relativeDifference(final float value, final float reference) {
        return Math.abs((value - reference) / reference);
    }

    private static int[] factorials = { 1, 1, 2, 6 };

    public static float hypotenuse(final float a, final float b) {
        return (float) Math.sqrt(a * a + b * b);
    }

    public static float sin(final float val){
        return (float) Math.sin(val);
    }
    public static float cos(final float val){
        return (float) Math.cos(val);
    }
    public static float tan(final float val){
        return (float) Math.tan(val);
    }
    public static float asin(final float val){
        return (float) Math.asin(val);
    }
    public static float acos(final float val){
        return (float) Math.acos(val);
    }
    public static float atan(final float val){
        return (float) Math.atan(val);
    }
    public static float atan2(final float val1,final float val2){
        return (float) Math.atan2(val1,val2);
    }

    public static float sqrt(final float val){
        return (float) Math.sqrt(val);
    }

    public static float pow(final float val,final float power){
        return (float) Math.pow(val,power);
    }

    public static float predict(final float dt, final float... derivatives) {
        float predictionAddition = derivatives[0];
        for (int i = 1; i < derivatives.length; i++) {
            predictionAddition += (float) Math.pow(dt, i) * derivatives[i] / factorials[i];
        }
        return predictionAddition;
    }

    public static Vector predict(final float dt, final Vector... derivatives) {
        Vector predictionAddition = derivatives[0];
        for (int i = 1; i < derivatives.length; i++) {
            final Vector currentDerivativeAddition = derivatives[i].scale((float) Math.pow(dt, i) / factorials[i]);
            predictionAddition = predictionAddition.add(currentDerivativeAddition);
        }
        return predictionAddition;
    }

    public static Pose2D predict(final float dt, final Pose2D... derivatives) {
        Pose2D predictionAddition = derivatives[0];
        for (int i = 1; i < derivatives.length; i++) {
            final Pose2D currentDerivativeAddition = derivatives[i].scale((float) Math.pow(dt, i) / factorials[i]);
            predictionAddition = predictionAddition.add(currentDerivativeAddition);
        }
        return predictionAddition;
    }


    public static float average(final float... values){
        float sum = 0;
        for (float value : values) {
            sum += value;
        }
        return sum / values.length;
    }
    public static float average(final int... values){
        float sum = 0;
        for (float value : values) {
            sum += value;
        }
        return sum / values.length;
    }
    public static Vector average(final Vector... values){
        float sumX = 0;
        float sumY = 0;
        for (Vector value : values) {
            sumX += value.x;
            sumY += value.y;

        }
        return new Vector(sumX,sumY).scale((float) 1 / values.length);
    }
    public static Pose2D average(final Pose2D... values) {
        float sumX = 0;
        float sumY = 0;
        float sumSin = 0;
        float sumCos = 0;

        for (Pose2D value : values) {
            sumX += value.translation.x;
            sumY += value.translation.y;
            sumSin += (float) Math.sin(value.rotation);
            sumCos += (float) Math.cos(value.rotation);
        }

        float avgX = sumX / values.length;
        float avgY = sumY / values.length;
        float avgHeading = (float) Math.atan2(sumSin, sumCos);

        return new Pose2D(avgX, avgY, avgHeading);
    }

    public static Pose3D average(final Pose3D... values){
        float sumZ = 0;
        float sumX = 0;
        float sumY = 0;
        float sumHeading = 0;
        float sumPitch = 0;
        for (Pose3D value : values) {
            sumZ += value.translation.z;
            sumX += value.translation.x;
            sumY += value.translation.y;
            sumHeading += value.rotation.z;
            sumPitch += value.rotation.y;
        }
        final Pose2D pose2D = new Pose2D(sumX,sumY,sumHeading).scale((float) 1 / values.length);
        return Pose3D.from2D(pose2D,sumZ / values.length,sumPitch / values.length);
    }

    public static float log(final float val){
        return (float) Math.log(val);
    }

    public static float approach(float current, float target, float delta) {
        if (current < target) {
            return Math.min(current + delta, target);
        } else {
            return Math.max(current - delta, target);
        }
    }




}