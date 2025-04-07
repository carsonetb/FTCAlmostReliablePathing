package Opmodes.Auto.CustomLocalizer.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Opmodes.Auto.CustomLocalizer.DynamicPathFollower;
import Opmodes.Auto.CustomLocalizer.PathFollowerUpdateError;
import Opmodes.Auto.CustomLocalizer.PathSegments.LineSegment;
import Util.Vector2;

@Autonomous(name = "SquareTest", group = "Auto")
public class SquareTest extends LinearOpMode {
    DynamicPathFollower pathSequenceFollower;

    public void runOpMode() throws InterruptedException {
        pathSequenceFollower = new DynamicPathFollower(hardwareMap, new Vector2(0, 0), 0, telemetry);

        waitForStart();

        while (!isStopRequested()) {
            PathFollowerUpdateError out = pathSequenceFollower.update();

            if (out == PathFollowerUpdateError.NO_PATH) {
                pathSequenceFollower.forward(24);
                pathSequenceFollower.turn(90);
            }

            telemetry.update();
        }
    }
}
