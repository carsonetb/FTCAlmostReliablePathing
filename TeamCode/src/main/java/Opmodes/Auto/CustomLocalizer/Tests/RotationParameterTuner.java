package Opmodes.Auto.CustomLocalizer.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Opmodes.Auto.CustomLocalizer.Constants;
import Opmodes.Auto.CustomLocalizer.PIDFFollower;
import Util.Vector2;

@Autonomous(name = "RotationParameterTuner", group = "Auto")
public class RotationParameterTuner extends LinearOpMode {
    PIDFFollower pathFollower;

    double previousPosition;
    double previousTime;

    double customHeadingkV = 0;

    boolean successVel = false;

    @Override
    public void runOpMode() throws InterruptedException {
        pathFollower = new PIDFFollower(hardwareMap, new Vector2(0, 0), 0, telemetry);

        waitForStart();

        previousPosition = pathFollower.getLocalizer().getAngle();
        previousTime = time;

        while (!isStopRequested() && !successVel) {
            pathFollower.getLocalizer().update();

            double deltaTime = Math.abs(time - previousTime);

            double current_position = pathFollower.getLocalizer().getAngle();
            double deltaMovement = Math.abs(current_position - previousPosition);
            double velocity = deltaMovement / deltaTime; // radians/s
            double maxAngVelRadians = Math.toRadians(Constants.testMaxAngVel);

            if (Math.abs(Math.toDegrees(velocity) - Constants.testMaxAngVel) < 0.1) {
                successVel = true;
                continue;
            }

            if (velocity < maxAngVelRadians) {
                customHeadingkV += 0.0002;
            }
            if (velocity > maxAngVelRadians) {
                customHeadingkV -= 0.0002;
            }

            previousPosition = current_position;
            previousTime = time;
            pathFollower.updateFeedforwardOnly(new Vector2(0, 0), new Vector2(0, 0), maxAngVelRadians, 0, 0, 0, customHeadingkV);

            telemetry.addData("Status", "Tuning kV");
            telemetry.addData("Current kV", customHeadingkV);
            telemetry.addData("vel * kv", maxAngVelRadians * customHeadingkV);
            telemetry.addData("Current velocity", Math.toDegrees(velocity));

            telemetry.update();
        }

        pathFollower.updateFeedforwardOnly(new Vector2(0, 0), new Vector2(0, 0), 0, 0, 0, 0, 0);

        for (int i = 0; i < 5000; i++) {
            Thread.sleep(1);
            telemetry.addLine("Final kV: " + customHeadingkV);
            telemetry.update();
        }
    }
}