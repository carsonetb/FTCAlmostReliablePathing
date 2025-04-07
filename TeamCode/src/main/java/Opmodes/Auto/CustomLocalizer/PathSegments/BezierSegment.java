package Opmodes.Auto.CustomLocalizer.PathSegments;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Util.Pose2D;
import Util.Vector2;
import Opmodes.Auto.CustomLocalizer.Constants;

public class BezierSegment extends PathSegment {
    private final Vector2 p1;
    private final Vector2 p2;
    private final Vector2 p3;
    private final Vector2 p4;
    private double splineLength = 0;
    private double distanceT = 0;
    private double distanceTraveled = 0;
    private double currentLinearVelocity = 0;
    private boolean alreadySlowedDown = false;
    private boolean firstUpdate = true;
    private double lastTime;
    Telemetry telemetry;

    public BezierSegment(Pose2D startPos, Pose2D endPos, double anchorLength, Telemetry telemetry) {
        super(startPos, endPos);
        p1 = startPos.getVector();
        p2 = startPos.getVector().add(Vector2.fromAngle(startPos.heading - Math.toRadians(90)).multiply(anchorLength));
        p4 = endPos.getVector();
        p3 = endPos.getVector().subtract(Vector2.fromAngle(endPos.heading - Math.toRadians(90)).multiply(anchorLength));

        Vector2 lastPos = startPos.getVector();
        for (double dist = 0; dist < 1; dist += Constants.bezierPrecision) {
            Vector2 currentPos = bezierPoint(dist);
            splineLength += lastPos.distanceTo(currentPos);
            lastPos = currentPos;
        }

        lastTime = System.currentTimeMillis() / 1000.0;
        this.telemetry = telemetry;
    }

    private Vector2 bezierPoint(double t) {
        Vector2 q1 = p1.lerp(p2, t);
        Vector2 q2 = p2.lerp(p3, t);
        Vector2 q3 = p3.lerp(p4, t);

        Vector2 r1 = q1.lerp(q2, t);
        Vector2 r2 = q2.lerp(q3, t);

        return r1.lerp(r2, t);
    }

    private Vector2 bezierDerivative(double t) {
        // 3 * (1 - t)^2 * (p2 - p1) + 6 * (1 - t) * t * (p3 - p2) + 3 * t^2 * (p4 - p3)
        // But java is stupid.
        return       p2.subtract(p1).multiply(3 * Math.pow(1 - t, 2))
                .add(p3.subtract(p2).multiply(6 * (1 - t) * t))
                .add(p4.subtract(p3).multiply(3 * Math.pow(t, 2)));
    }

    @Override
    public void update() {
        if (firstUpdate) {
            firstUpdate = false;
            lastTime = System.currentTimeMillis() / 1000.0;
            return;
        }

        double delta = (System.currentTimeMillis() / 1000.0) - lastTime;
        lastTime = System.currentTimeMillis() / 1000.0;

        double distanceRemaining = splineLength - distanceTraveled;

        Vector2 derivative = bezierDerivative(distanceT);
        distanceT += (currentLinearVelocity / derivative.length()) * delta;
        currentPos.setVector(bezierPoint(distanceT));
        distanceTraveled += currentLinearVelocity * delta;

        telemetry.addData("distanceTraveled", distanceTraveled);
        telemetry.addData("distanceRemaining", distanceRemaining);
        telemetry.addData("bezierlength", splineLength);
        telemetry.addData("currentposx", currentPos.getVector().x);
        telemetry.addData("currentposy", currentPos.getVector().y);
        telemetry.addData("derivativelength", derivative.length());
        telemetry.addData("velocity", currentLinearVelocity);

        Vector2 direction = derivative.normalized();
        double currentMaxAccelDistance = Math.pow(currentLinearVelocity, 2) / (2 * Constants.maxAccel);

        if (distanceRemaining < 0.1) {
            currentLinearVelocity = 0;
            currentVelocity.setVector(direction.multiply(0));
            currentAcceleration.setVector(direction.multiply(-currentLinearVelocity));
            isFinished = true;
            return;
        }

        // Slowing down
        if (distanceRemaining < currentMaxAccelDistance) {
            telemetry.addData("slwoing down", true);
            alreadySlowedDown = true;
            currentLinearVelocity -= Constants.maxAccel * delta;
            currentVelocity.setVector(direction.multiply(currentLinearVelocity));
            currentAcceleration.setVector(direction.multiply(-Constants.maxAccel));
        }

        // Speeding up
        else if (currentLinearVelocity < Constants.maxVel && !alreadySlowedDown) {
            telemetry.addData("speeding up", true);
            if (Constants.maxVel - currentLinearVelocity < Constants.maxAccel * delta) {
                currentLinearVelocity = Constants.maxVel;
                currentVelocity.setVector(direction.multiply(Constants.maxVel));
                currentAcceleration.setVector(direction.multiply(0));
            }
            else {
                currentLinearVelocity += Constants.maxAccel * delta;
                currentVelocity.setVector(direction.multiply(currentLinearVelocity));
                currentAcceleration.setVector(direction.multiply(Constants.maxAccel));
            }
        }

        // TODO: Heading
        currentPos.heading = 0;
    }
}