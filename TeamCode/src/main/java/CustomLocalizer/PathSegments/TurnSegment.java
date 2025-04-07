package CustomLocalizer.PathSegments;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import CustomLocalizer.Constants;
import Util.Pose2D;
import Util.Vector2;

public class TurnSegment extends PathSegment {
    Telemetry telemetry;
    double lastTime;
    boolean alreadySlowedDown = false;
    boolean alreadyGoingStraight = false;
    boolean firstUpdate = true;
    double previousDirection;

    public TurnSegment(Vector2 pos, double startHeading, double endHeading, Telemetry telemetry) {
        super(new Pose2D(pos.x, pos.y, startHeading), new Pose2D(pos.x, pos.y, endHeading));
        this.lastTime = System.currentTimeMillis() / 1000.0;
        this.telemetry = telemetry;
        this.currentVelocity.heading = 0;
        this.currentAcceleration.heading = 0;
    }

    public void update() {
        if (firstUpdate) {
            firstUpdate = false;
            this.lastTime = System.currentTimeMillis() / 1000.0;
            return;
        }

        double delta = (System.currentTimeMillis() / 1000.0) - lastTime;
        lastTime = System.currentTimeMillis() / 1000.0;

        currentPos.heading += currentVelocity.heading * delta;
        currentPos.setVector(endPos.getVector());

        double difference = endPos.heading - currentPos.heading;
        difference = (difference + 180) % 360 - 180;

        double distance = Math.abs(endPos.heading - currentPos.heading);
        double currentMaxAccelDistance = Math.pow(currentVelocity.heading, 2) / (2 * Constants.maxAngAccel);
        double currentDirection = Math.signum(difference);
        double currentSpeed = Math.abs(currentVelocity.heading);

        if (previousDirection != 0 && currentDirection != previousDirection) {
            currentAcceleration.heading = -currentVelocity.heading;
            currentVelocity.heading = 0;
            isFinished = true;
            return;
        }

        if (distance <= currentMaxAccelDistance) {
            telemetry.addLine("slowing down");
            alreadySlowedDown = true;
            currentVelocity.heading -= currentDirection * Constants.maxAngAccel * delta;
            currentAcceleration.heading = 0;
        }

        else if (currentVelocity.heading < Constants.maxAngVel && !alreadySlowedDown && !alreadyGoingStraight) {
            telemetry.addLine("speeding up");
            if (Constants.maxAngVel - currentSpeed < Constants.maxAngAccel * delta) {
                currentVelocity.heading = currentDirection * Constants.maxAngVel;
                currentAcceleration.heading = 0;
            }
            else {
                currentVelocity.heading += currentDirection * Constants.maxAngAccel * delta;
                currentAcceleration.heading = currentDirection * Constants.maxAngAccel;
            }
        }

        telemetry.addData("currentVelocity", currentVelocity.heading);
        telemetry.addData("currentPosition", currentPos.heading);
        telemetry.addData("currentAccelration", currentAcceleration.heading);
        telemetry.addData("endHeading", endPos.heading);
        telemetry.addData("difference", difference);

        previousDirection = currentDirection;
    }
}
