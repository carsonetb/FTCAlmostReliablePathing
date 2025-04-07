package Opmodes.Auto.CustomLocalizer.PathSegments;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import Opmodes.Auto.CustomLocalizer.Constants;
import Util.Pose2D;

public class LineSegment extends LineSegmentConstantHeading {
    private double lastTime2;
    boolean alreadySlowedDown = false;
    boolean alreadyGoingStraight = false;
    boolean firstUpdate = true;
    boolean readyToFinish = false;
    double previousDirection;
    Telemetry telemetry;

    public LineSegment(Pose2D startPos, Pose2D endPos, Telemetry telemetry) {
        super(startPos, endPos, telemetry);
        this.lastTime2 = System.currentTimeMillis() / 1000.0;
        this.currentVelocity.heading = 0;
        this.currentAcceleration.heading = 0;
        this.telemetry = telemetry;
    }

    public void update() {
        super.update();

        if (firstUpdate) {
            firstUpdate = false;
            lastTime2 = System.currentTimeMillis() / 1000.0;
            return;
        }

        double delta = (System.currentTimeMillis() / 1000.0) - lastTime2;
        lastTime2 = System.currentTimeMillis() / 1000.0;

        currentPos.heading += currentVelocity.heading * delta;

        double difference = endPos.heading - currentPos.heading;
        difference = (difference + 180) % 360 - 180;

        double distance = Math.abs(endPos.heading - currentPos.heading);
        double currentMaxAccelDistance = Math.pow(currentVelocity.heading, 2) / (2 * Constants.maxAngAccel);
        double currentDirection = Math.signum(difference);
        double currentSpeed = Math.abs(currentVelocity.heading);

        telemetry.addData("readytofinsih", readyToFinish);
        telemetry.addData("isfinished", isFinished);

        if (previousDirection != 0 && currentDirection != previousDirection) {
            currentAcceleration.heading = -currentVelocity.heading;
            currentVelocity.heading = 0;
            readyToFinish = true;
            return;
        }
        if (readyToFinish) {
            currentAcceleration.heading = 0;
            currentVelocity.heading = 0;
            return;
        }
        if (isFinished) {
            isFinished = false;
        }

        if (distance <= currentMaxAccelDistance) {
            alreadySlowedDown = true;
            currentVelocity.heading -= currentDirection * Constants.maxAngAccel * delta;
            currentAcceleration.heading = 0;
        }

        else if (currentVelocity.heading < Constants.maxAngVel && !alreadySlowedDown && !alreadyGoingStraight) {
            if (Constants.maxAngVel - currentSpeed < Constants.maxAngAccel * delta) {
                currentVelocity.heading = currentDirection * Constants.maxAngVel;
                currentAcceleration.heading = 0;
            }
            else {
                currentVelocity.heading += currentDirection * Constants.maxAngAccel * delta;
                currentAcceleration.heading = currentDirection * Constants.maxAngAccel;
            }
        }

        previousDirection = currentDirection;
    }
}
