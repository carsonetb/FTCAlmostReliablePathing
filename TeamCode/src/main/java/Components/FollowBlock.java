package Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import Opmodes.Auto.Pipelines.EdgeDetectionProcessPipeline;
import Util.Vector2;

enum MovementState {
    ALIGNING,
    MOVING,
}

@Config
public class FollowBlock {
    public OpenCvWebcam webcam;
    EdgeDetectionProcessPipeline pipeline;
    boolean foundBlock = false;
    Telemetry telemetry;
    Vector2 previousVelocity = new Vector2(0, 0);
    MovementState state = MovementState.ALIGNING;
    int timesFoundBlock;

    public static int timesAlignedStartMovingForward = 10;

    public FollowBlock(String webcamName, Telemetry robot_telemetry, HardwareMap hardwareMap) {
        this.telemetry = robot_telemetry;

        String packageName = hardwareMap.appContext.getPackageName();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", packageName);

        pipeline = new EdgeDetectionProcessPipeline();

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error: " + errorCode);
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        telemetry.addLine("Camera initialized successfully if there were no errors above.");
    }

    public Vector2 getMovementDirection() {
        if (pipeline.notFound) {
            return new Vector2(0, 0);
        }

        float multiplier = Math.min(Math.max(0.2f, Math.abs(pipeline.center) / 130.0f), 0.4f);
        Vector2 targetVelocity = new Vector2(0, 0);

        switch (state) {
            case ALIGNING: {
                if (pipeline.center < -3) {
                    foundBlock = false;
                    targetVelocity = new Vector2(0, -multiplier);
                } else if (pipeline.center > 3) {
                    foundBlock = false;
                    targetVelocity = new Vector2(0, multiplier);
                } else {
                    timesFoundBlock++;
                }

                if (timesFoundBlock == timesAlignedStartMovingForward) {
                    state = MovementState.MOVING;
                }

                break;
            }
            case MOVING: {
//                if (pipeline.center < -3) {
//                    foundBlock = false;
//                    targetVelocity = new Vector2(0, -multiplier);
//                } else if (pipeline.center > 3) {
//                    foundBlock = false;
//                    targetVelocity = new Vector2(0, multiplier);
//                }

                targetVelocity.x = 0.4f;

                break;
            }
        }

        telemetry.addData("Move Toward Specimen Velocity", targetVelocity);
        telemetry.addData("Estimate specimen center", pipeline.center);
        telemetry.addData("Times saw block", timesFoundBlock);
        telemetry.update();

        // Smoothing!
        previousVelocity = previousVelocity.add(targetVelocity.subtract(previousVelocity).multiply(0.2f));
        return previousVelocity;
    }
}
