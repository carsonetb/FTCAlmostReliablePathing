package CustomLocalizer.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import CustomLocalizer.DynamicPathFollower;
import CustomLocalizer.PathFollowerUpdateError;
import Util.Pose2D;
import Util.Vector2;

@Autonomous(name = "LineSegmentTest", group = "Auto")
public class LineSegmentTest extends LinearOpMode {
    DynamicPathFollower pathSequenceFollower;

    public void runOpMode() throws InterruptedException {
        pathSequenceFollower = new DynamicPathFollower(hardwareMap, new Vector2(0, 0), 0, telemetry);

        waitForStart();

        double a = 90;

        while (!isStopRequested()) {
            PathFollowerUpdateError out = pathSequenceFollower.update();

            if (out == PathFollowerUpdateError.NO_PATH) {
                a += 90;
                if (a == 360) {
                    a = 0;
                }
                pathSequenceFollower.lineTo(new Pose2D(0, -24, a));
                pathSequenceFollower.lineTo(new Pose2D(0, 0, a - 90));
            }

            telemetry.update();
        }
    }
}
