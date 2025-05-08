package org.firstinspires.ftc.teamcode.OrbitUtils;

import org.firstinspires.ftc.teamcode.robotData.Constants;

public class Vector3D {
    public float x;
    public float y;
    public float z;
    private static final Vector3D zero = new Vector3D(0,0,0);
    private static final Vector3D INF = new Vector3D(Constants.INF,Constants.INF,Constants.INF);

    public Vector3D(float x, float y, float z){
    this.x = x;
    this.y = y;
    this.z = z;
    }

    public static Vector3D zero(){
        return zero;
    }


    public static Vector3D INF(){
        return INF;
    }

    public static Vector3D from2D(final Vector xy, final float z){
        return new Vector3D(xy.x, xy.y, z);
    }

    public Vector xy() {
        return new Vector(x, y);
    }

    public Vector3D add(final Vector3D otherVector){
        return new Vector3D(x + otherVector.x, y + otherVector.y , z + otherVector.z);
    }

    public Vector3D add(final Vector otherVector){
        return new Vector3D(x + otherVector.x, y + otherVector.y, z);
    }

    public Vector3D subtract(final Vector3D otherVector){
        return new Vector3D(x - otherVector.x , y - otherVector.y, z - otherVector.z);
    }

    public Vector3D subtract(final Vector otherVector){
        return new Vector3D(x - otherVector.x , y - otherVector.y, z);
    }

    public Vector3D scale(final float scalingFactor){
        return new Vector3D(x * scalingFactor, y * scalingFactor, z * scalingFactor);
    }

    public Vector3D dev(final float devFactor){
        return new Vector3D(x / devFactor, y / devFactor, z / devFactor);
    }

    public Vector3D abs(){
        return new Vector3D(Math.abs(x),Math.abs(y),Math.abs(z));
    }

    public Vector3D max(final Vector3D a, final Vector3D b){
        return new Vector3D(Math.max(a.x,b.x),Math.max(a.y,b.y),Math.max(a.z,b.z));
    }

    public Vector3D min(final Vector3D a, final Vector3D b){
        return new Vector3D(Math.min(a.x,b.x),Math.min(a.y,b.y),Math.min(a.z,b.z));
    }

    public Vector3D sameXYZ(final float value){
        return new Vector3D(value,value,value);
    }

    public Vector3D longest(final Vector3D a, final Vector3D b){
        return a.norm() > b.norm() ? a : b;
    }

    public Vector3D shortest(final Vector3D a, final Vector3D b){
        return a.norm() < b.norm() ? a : b;
    }

    public boolean equals(final Vector3D otherVector){
        return x == otherVector.x && y == otherVector.y && z == otherVector.z;
    }

    public Vector3D unit() {
        if (x == 0 && y == 0 && z == 0) {
            return this;
        } else {
            return scale(1 / norm());
        }
    }

    public float dotProduct(final Vector3D other) {
        return x * other.x + y * other.y + z * other.z;
    }




    public static Vector3D fromSizeYawPitch(final float size, final float yaw, final float pitch) {
        final float vectorX = (float) (Math.cos(yaw) * Math.cos(pitch)) * size;
        final float vectorY = (float) (Math.sin(yaw) * Math.cos(pitch)) * size;
        final float vectorZ = (float) Math.sin(pitch) * size;
        return new Vector3D(vectorX, vectorY, vectorZ);
    }

    public float norm() {
        return (float) (Math.sqrt(x * x + y * y + z * z));
    }

    public float getYaw() {
        return xy().getAngle();
    }

    public float getPitch() {
        return (float) (Math.atan2(z, xy().norm()));
    }


    public Vector3D rotateRoll(final float theta) {
        return new Vector3D(x, (float) (y * Math.cos(theta) - z * Math.sin(theta)),
                (float) (z * Math.cos(theta) + y * Math.sin(theta)));
    }

    public Vector3D rotatePitch(final float theta) {
        return new Vector3D((float) (x * Math.cos(theta) + z * Math.sin(theta)), y,
                (float) (z * Math.cos(theta) - x * Math.sin(theta)));
    }

    public Vector3D rotateYaw(final float theta) {
        return new Vector3D((float) (x * Math.cos(theta) - y * Math.sin(theta)),
                (float) (y * Math.cos(theta) + x * Math.sin(theta)),
                z);
    }

    public Pose2D getPose2D() {
        return new Pose2D(new Vector(x, y), getYaw());
    }

    public static float angleDifference(final Vector3D a, final Vector3D b) {
        final float normA = a.norm();
        final float normB = b.norm();

        if (normA == 0 || normB == 0)
            return 0;

        return (float) Math.acos(a.dotProduct(b) / (normA * normB));
    }



    @Override
    public String toString() {
        return "x: " + x + " y: " + y + " z: " + z;
    }


}
