package Opmodes.Auto.CustomLocalizer.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Opmodes.Auto.CustomLocalizer.Constants;
import Opmodes.Auto.CustomLocalizer.PIDFFollower;
import Util.Vector2;

@Autonomous(name = "ParameterTuner", group = "Auto")
public class ParameterTuner extends LinearOpMode {
    PIDFFollower pathFollower;

    Vector2 previousPosition;
    double previousVelocity;
    double previousTime;

    double customkV = 0;
    double customkA = 0;

    boolean successVel = false;
    boolean successAccel = false;

    @Override
    public void runOpMode() throws InterruptedException {
        pathFollower = new PIDFFollower(hardwareMap, new Vector2(0, 0), 0, telemetry);

        waitForStart();

        previousPosition = pathFollower.getLocalizer().getPos();
        previousTime = time;

        while (!isStopRequested() && !successVel) {
            pathFollower.getLocalizer().update();

            double deltaTime = Math.abs(time - previousTime);

            Vector2 current_position = pathFollower.getLocalizer().getPos();
            double deltaMovement = Math.abs(current_position.y - previousPosition.y);
            double velocity = deltaMovement / deltaTime; // in/s

            if (Math.abs(velocity - Constants.testMaxVel) < 0.02) {
                successVel = true;
                continue;
            }

            if (velocity < Constants.testMaxVel) {
                customkV += 0.0002;
            }
            if (velocity > Constants.testMaxVel) {
                customkV -= 0.0002;
            }

            previousPosition = current_position;
            previousTime = time;
            pathFollower.updateFeedforwardOnly(new Vector2(0, Constants.testMaxVel), new Vector2(0, 0), 0, 0, customkV, customkA, 0);

            telemetry.addData("Status", "Tuning kV");
            telemetry.addData("Current kV", customkV);
            telemetry.addData("Current velocity", velocity);

            telemetry.update();
        }

        telemetry.addLine("Final kV: " + customkV);

        pathFollower.updateFeedforwardOnly(new Vector2(0, 0), new Vector2(0, 0), 0, 0, customkV, customkA, 0);

//        previousVelocity = 0;
//        previousPosition = new Vector2(0, 0);
//
//        while (!isStopRequested() && !successAccel) {
//            double deltaTime = Math.abs(time - previousTime);
//
//            Vector2 current_position = pathFollower.getLocalizer().getPos();
//            double deltaMovement = Math.abs(current_position.y - previousPosition.y);
//            double current_velocity = deltaMovement * deltaTime; // in/s
//            double acceleration = Math.abs(current_velocity - previousVelocity) * deltaTime;
//
//            if (Math.abs(acceleration - Constants.testMaxAccel) < 0.02) {
//                successAccel = true;
//                continue;
//            }
//            if (current_velocity > Constants.testMaxVel) {
//                telemetry.addLine("Reached max velocity while tuning max acceleration. Retune likely necessary.");
//                continue;
//            }
//
//            if (acceleration < Constants.testMaxAccel) {
//                customkA += 0.001;
//            }
//            if (acceleration > Constants.testMaxAccel) {
//                customkA -= 0.001;
//            }
//
//            previousPosition = current_position;
//            previousVelocity = current_velocity;
//            previousTime = time;
//            pathFollower.updateFeedforwardOnly(new Vector2(0, current_velocity), new Vector2(0, Constants.testMaxAccel), 0, 0, customkV, customkA, 0);
//
//            telemetry.addData("Status", "Tuning kA");
//            telemetry.addData("Current kA", customkA);
//            telemetry.addData("Current kV", customkV);
//
//            telemetry.update();
//        }
//
//        telemetry.addLine("Final kA: " + customkA);
//
//        pathFollower.updateFeedforwardOnly(new Vector2(0, 0), new Vector2(0, 0), 0, 0, customkV, customkA, 0);
//
        Thread.sleep(5000);
    }
}
