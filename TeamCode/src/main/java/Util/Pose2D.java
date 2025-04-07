package Util;

public class Pose2D {
    public double x;
    public double y;
    public double heading;

    public Pose2D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public void setVector(Vector2 set) {
        x = set.x;
        y = set.y;
    }

    public Vector2 getVector() {
        return new Vector2(x, y);
    }

    public void setHeading(double newHeading) {
        while (newHeading > 360) {
            newHeading -= 360;
        }
        while (newHeading < 0) {
            newHeading += 360;
        }
    }
}
