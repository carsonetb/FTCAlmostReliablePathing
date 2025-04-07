package CustomLocalizer.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import CustomLocalizer.DynamicPathFollower;
import CustomLocalizer.PathFollowerUpdateError;
import Util.Vector2;

@Autonomous(name = "HeartDrawer", group = "Auto")
public class HeartDrawer extends LinearOpMode {
    DynamicPathFollower pathSequenceFollower;

    public void runOpMode() throws InterruptedException {
        pathSequenceFollower = new DynamicPathFollower(hardwareMap, new Vector2(0, 0), 0, telemetry);

        waitForStart();
        pathSequenceFollower.lineToConstantHeading(new Vector2(0, 3));
        pathSequenceFollower.waits(1);
        pathSequenceFollower.lineToConstantHeading(new Vector2(3, 3));
        pathSequenceFollower.waits(1);
        pathSequenceFollower.lineToConstantHeading(new Vector2(3, 6));
        pathSequenceFollower.waits(1);
        pathSequenceFollower.lineToConstantHeading(new Vector2(6, 6));
        pathSequenceFollower.waits(1);
        pathSequenceFollower.lineToConstantHeading(new Vector2(6, 0));
        pathSequenceFollower.waits(1);
        pathSequenceFollower.lineToConstantHeading(new Vector2(0, 0));
        pathSequenceFollower.waits(1);

        while (!isStopRequested()) {
            PathFollowerUpdateError out = pathSequenceFollower.update();

            if (out == PathFollowerUpdateError.NO_PATH) {
                return;
            }

            telemetry.update();
        }
    }
}
