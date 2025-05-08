package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.OrbitUtils.MathFuncs;
import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.firstinspires.ftc.teamcode.OrbitUtils.Vector;

import java.util.ArrayList;
import java.util.List;

public class OrbitPosMap {

    private final List<CircularDeadBend> circularDeadBends = new ArrayList<>();
    private final List<RectangularDeadband> rectangularDeadBends = new ArrayList<>();

    // Enum to specify the type of deadband
    public enum DeadBendType {
        CIRCLE, CUBE
    }


    public void addDeadBend(DeadBendType type, float radius, Pose2D... positions) {
        switch (type) {
            case CIRCLE:
                if (positions.length != 1) {
                    throw new IllegalArgumentException("CIRCLE deadband requires exactly one Pose2D (center).");
                }
                circularDeadBends.add(new CircularDeadBend(positions[0], radius));
                break;

            case CUBE:
                if (positions.length != 2) {
                    throw new IllegalArgumentException("CUBE deadband requires exactly two Pose2D objects (bottom-left and top-right).");
                }
                rectangularDeadBends.add(new RectangularDeadband(positions[0], positions[1]));
                break;

            default:
                throw new IllegalArgumentException("Invalid DeadbandType specified.");
        }
    }

    public boolean isInDeadBend(Pose2D position) {
        // Check circular deadbands
        for (CircularDeadBend circle : circularDeadBends) {
            if (circle.isInDeadBend(position)) {
                return true;
            }
        }
        // Check rectangular deadbands
        for (RectangularDeadband rectangle : rectangularDeadBends) {
            if (rectangle.isInDeadBend(position)) {
                return true;
            }
        }
        return false;
    }

    // Circular Deadband Class
    private static class CircularDeadBend {
        private final Pose2D center;
        private final float radius;

        public CircularDeadBend(Pose2D center, float radius) {
            this.center = center;
            this.radius = radius;
        }

        public boolean isInDeadBend(Pose2D position) {
            float distanceSquared = MathFuncs.pow(center.translation.subtract(position.translation).norm(), 2);
            return distanceSquared <= radius * radius;
        }
    }

    // Rectangular Deadband Class
    private static class RectangularDeadband {
        private final Pose2D bottomLeft;
        private final Pose2D topRight;

        public RectangularDeadband(Pose2D bottomLeft, Pose2D topRight) {
            this.bottomLeft = bottomLeft;
            this.topRight = topRight;
        }

        public boolean isInDeadBend(Pose2D position) {
            float x = position.translation.x;
            float y = position.translation.y;
            return x >= bottomLeft.translation.x && x <= topRight.translation.x &&
                    y >= bottomLeft.translation.y && y <= topRight.translation.y;
        }
    }

    public void clearDeadBends() {
        circularDeadBends.clear();
        rectangularDeadBends.clear();
    }
    public void clearDeadBend(final int index,final DeadBendType type){
        switch (type){
            case CIRCLE:
                circularDeadBends.remove(index);
                break;
            case CUBE:
                rectangularDeadBends.remove(index);
                break;
        }
    }

    public Pose2D[] adjustPath(Pose2D... wayPoints) {

        for (int i = 0; i < wayPoints.length - 1; i++) {
            wayPoints[i] = adjustPoint(wayPoints[i]);
        }

        return wayPoints;
    }
    public Pose2D adjustPoint(Pose2D wayPoint) {
        for (CircularDeadBend circle : circularDeadBends) {
            if (circle.isInDeadBend(wayPoint)) {
                Vector delta = wayPoint.translation.subtract(circle.center.translation);
                wayPoint = wayPoint.add(new Pose2D(Vector.fromAngleAndRadius(delta.getAngle(),circle.radius), 0));
                return adjustPoint(wayPoint);  // Ensure the adjustment propagates
            }
        }
        for (RectangularDeadband rectangle : rectangularDeadBends) {
            if (rectangle.isInDeadBend(wayPoint)) {
                float newX = Math.min(Math.max(wayPoint.translation.x, rectangle.bottomLeft.translation.x), rectangle.topRight.translation.x);
                float newY = Math.min(Math.max(wayPoint.translation.y, rectangle.bottomLeft.translation.y), rectangle.topRight.translation.y);
                wayPoint = new Pose2D(new Vector(newX, newY), wayPoint.rotation);
                return adjustPoint(wayPoint);  // Ensure the adjustment propagates
            }
        }
        return wayPoint;
    }


}



