package CustomLocalizer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import CustomLocalizer.PathSegments.BezierSegment;
import CustomLocalizer.PathSegments.CodeSegment;
import CustomLocalizer.PathSegments.LineSegment;
import CustomLocalizer.PathSegments.LineSegmentConstantHeading;
import CustomLocalizer.PathSegments.PathSegment;
import CustomLocalizer.PathSegments.TurnSegment;
import CustomLocalizer.PathSegments.WaitSegment;
import Util.CustomCallable;
import Util.Pose2D;
import Util.Vector2;

// This path follower is dynamic meaning that it can be updated at runtime. The functionality relies
// on a path queue which empties out over time.
public class DynamicPathFollower {
    private static final double maxPathSegmentTime = 20000;
    PIDFFollower pointFollower;
    ArrayList<PathSegment> pathQueue = new ArrayList<>();
    boolean isSuspended = false;
    double timeStartedCurrentPathSegment = System.currentTimeMillis();
    Pose2D finalPos;
    private Telemetry telemetry;

    public DynamicPathFollower(HardwareMap hardwareMap, Vector2 initialPos, double initialHeading, Telemetry telemetry) {
        pointFollower = new PIDFFollower(hardwareMap, initialPos, initialHeading, telemetry);
        finalPos = new Pose2D(initialPos.x, initialPos.y, initialHeading);
        this.telemetry = telemetry;
    }

    public void addSegmentToQueue(PathSegment segment) {
        pathQueue.add(segment);
    }

    public PathFollowerUpdateError update() {
        pointFollower.localizer.update();

        telemetry.addData("real x", pointFollower.localizer.getPos().x);
        telemetry.addData("real y", pointFollower.localizer.getPos().y);

        if (pathQueue.isEmpty() || isSuspended) {
            return PathFollowerUpdateError.NO_PATH;
        }

        finalPos = pathQueue.get(pathQueue.size() - 1).endPos;

        PathSegment currentSegment = pathQueue.get(0);
        currentSegment.update();

        pointFollower.updatePIDF(
                currentSegment.getPos().getVector(),
                currentSegment.getVel(),
                currentSegment.getAccel(),
                Math.toRadians(currentSegment.getPos().heading),
                currentSegment.getHeadingVel(),
                currentSegment.getHeadingAccel()
        );

        if (currentSegment.isFinished) {
            pathQueue.remove(0);
        }

        return PathFollowerUpdateError.SUCCESS;
    }

    private void updateFinalPos() {
        finalPos = pathQueue.get(pathQueue.size() - 1).endPos;
    }

    public void forward(double distance) {
        Vector2 endVector = finalPos.getVector().add(Vector2.fromAngle(Math.toRadians(finalPos.heading + 90)).multiply(-distance));
        addPathSegment(new LineSegmentConstantHeading(finalPos, new Pose2D(endVector.x, endVector.y, finalPos.heading), telemetry));
    }

    public void back(double distance) {
        forward(-distance);
    }

    public void turn(double angle) {
        addPathSegment(new TurnSegment(finalPos.getVector(), finalPos.heading, finalPos.heading + angle, telemetry));
    }

    public void lineTo(Pose2D endPos) {
        addPathSegment(new LineSegment(finalPos, endPos, telemetry));
    }

    public void lineToConstantHeading(Vector2 endPos) {
        addPathSegment(new LineSegmentConstantHeading(finalPos, new Pose2D(endPos.x, endPos.y, finalPos.heading), telemetry));
    }

    public void splineTo(Pose2D endPos) {
        addPathSegment(new BezierSegment(finalPos, endPos, 10, telemetry));
    }

    public void waits(double time) {
        addPathSegment(new WaitSegment(finalPos, time));
    }

    public void function(CustomCallable call) {
        addPathSegment(new CodeSegment(finalPos, call));
    }

    public void suspendPathFollowing() {
        isSuspended = true;
    }

    public void resumePathFollowing() {
        isSuspended = false;
    }

    public void clearQueue() {
        pathQueue.clear();
    }

    public void addPathSegment(PathSegment toAdd) {
        pathQueue.add(toAdd);
        updateFinalPos();
    }

    public ArrayList<PathSegment> getQueue() {
        return pathQueue;
    }

    public void setQueue(ArrayList<PathSegment> newQueue) {
        pathQueue = newQueue;
    }
}
