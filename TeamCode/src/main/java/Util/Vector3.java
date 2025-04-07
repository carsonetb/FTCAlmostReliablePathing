package Util;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class Vector3 {
    public double x;
    public double y;
    public double z;

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3(Position from) {
        this.x = from.x;
        this.y = from.y;
        this.z = from.z;
    }

    public static String toString(Vector3 from) {
        return "(" + Double.toString(from.x) + ", " + Double.toString(from.y) + ", " + Double.toString(from.z) + ")";
    }

    public boolean equals(Vector3 a) {
        return this.x == a.x && this.y == a.y && this.z == a.z;
    }

    public double distanceTo(Vector3 to) {
        return Math.sqrt(Math.pow(this.z - to.z, 2) + Math.pow(this.y - to.y, 2) + Math.pow(this.x - to.x, 2));
    }

    public Vector3 add(Vector3 a) {
        return new Vector3(this.x + a.x, this.y + a.y, this.z + a.z);
    }

    public Vector3 add(double a) {
        return new Vector3(this.x + a, this.y + a, this.z + a);
    }

    public Vector3 multiply(Vector3 a) {
        return new Vector3(this.x * a.x, this.y * a.y, this.z * a.z);
    }

    public Vector3 multiply(double a) {
        return new Vector3(this.x * a, this.y * a, this.z * a);
    }

    public Vector3 subtract(Vector3 a) {
        return new Vector3(this.x - a.x, this.y - a.y, this.z - a.z);
    }

    public Vector3 subtract(double a) {
        return new Vector3(this.x - a, this.y - a, this.z - a);
    }

    public Vector3 divide(Vector3 a) {
        return new Vector3(this.x / a.x, this.y / a.y, this.z / a.z);
    }

    public Vector3 divide(double a) {
        return new Vector3(this.x / a, this.y / a, this.z / a);
    }

    public void rotate(double yaw, double pitch) {
        // Convert angles to radians
        double yawRad = Math.toRadians(yaw);
        double pitchRad = Math.toRadians(pitch);

        // Apply yaw rotation (around Y-axis)
        double cosYaw = Math.cos(yawRad);
        double sinYaw = Math.sin(yawRad);
        double newX = x * cosYaw - z * sinYaw;
        double newZ = x * sinYaw + z * cosYaw;

        // Apply pitch rotation (around X-axis)
        double cosPitch = Math.cos(pitchRad);
        double sinPitch = Math.sin(pitchRad);
        double newY = y * cosPitch - newZ * sinPitch;
        newZ = y * sinPitch + newZ * cosPitch;

        // Update vector components
        this.x = newX;
        this.y = newY;
        this.z = newZ;
    }
}
