package Opmodes.Auto.CustomLocalizer.PathSegments;

import Util.Pose2D;
import Util.Vector2;

public abstract class PathSegment {
    public final Pose2D startPos;
    public final Pose2D endPos;
    public Pose2D currentPos;
    public Pose2D currentVelocity = new Pose2D(0, 0, 0);
    public Pose2D currentAcceleration = new Pose2D(0, 0, 0);
    public boolean isFinished;

    protected PathSegment(Pose2D startPos, Pose2D endPos) {
        this.startPos = startPos;
        this.endPos = endPos;
        this.currentPos = startPos;
    }

    public abstract void update();

    public Pose2D getPos() {
        return currentPos;
    }

    public Vector2 getVel() {
        return currentVelocity.getVector();
    }

    public Vector2 getAccel() {
        return currentAcceleration.getVector();
    }

    public double getHeadingVel() {
        return Math.toRadians(currentVelocity.heading);
    }

    public double getHeadingAccel() {
        return Math.toRadians(currentAcceleration.heading);
    }
}
