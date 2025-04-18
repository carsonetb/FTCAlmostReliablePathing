package CustomLocalizer.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import CustomLocalizer.DynamicPathFollower;
import CustomLocalizer.PathFollowerUpdateError;
import Util.Pose2D;
import Util.Vector2;

@Autonomous(name = "CustomSplineTest", group = "Auto")
public class CustomSplineTest extends LinearOpMode {
    DynamicPathFollower pathSequenceFollower;

    public void runOpMode() throws InterruptedException {
        pathSequenceFollower = new DynamicPathFollower(hardwareMap, new Vector2(0, 0), 0, telemetry);

        waitForStart();
        pathSequenceFollower.splineTo(new Pose2D(-20, 0, 180));

        while (!isStopRequested()) {
            PathFollowerUpdateError out = pathSequenceFollower.update();

            if (out == PathFollowerUpdateError.NO_PATH) {
                return;
            }

            telemetry.update();
        }
    }
}
