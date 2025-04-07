package CustomLocalizer.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import CustomLocalizer.DynamicPathFollower;
import CustomLocalizer.PathFollowerUpdateError;
import Util.Vector2;

@Autonomous(name = "BackAndForthTest", group = "Auto")
public class BackAndForthTest extends LinearOpMode {
    DynamicPathFollower pathSequenceFollower;

    public void runOpMode() throws InterruptedException {
        pathSequenceFollower = new DynamicPathFollower(hardwareMap, new Vector2(0, 0), 0, telemetry);

        waitForStart();

        while (!isStopRequested()) {
            PathFollowerUpdateError out = pathSequenceFollower.update();

            if (out == PathFollowerUpdateError.NO_PATH) {
                pathSequenceFollower.back(36);
                pathSequenceFollower.forward(36);
            }

            telemetry.update();
        }
    }
}
