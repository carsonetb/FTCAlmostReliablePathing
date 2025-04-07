package CustomLocalizer.Tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import CustomLocalizer.Constants;
import CustomLocalizer.Drive;
import CustomLocalizer.Localizer;
import Util.Encoder;
import Util.Vector2;

@Autonomous(name = "CustomLocalizationTest", group = "Auto")
public class CustomLocalizationTest extends LinearOpMode {
    DcMotorEx leftFrontMotor;
    DcMotorEx leftRearMotor;
    DcMotorEx rightRearMotor;
    DcMotorEx rightFrontMotor;

    Encoder perpendicularEncoder;
    Encoder parallelEncoder;

    Drive mecanumDrive;
    Localizer localizer;

    IMU imu;

    private static final Vector2 initialPos = new Vector2(0, 0);
    private static final double initialHeading = 0;

    @Override
    public void runOpMode() {
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRearMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRearMotor = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                Constants.LOGO_FACING_DIR, Constants.USB_FACING_DIR));
        imu.initialize(parameters);

        localizer = new Localizer(perpendicularEncoder, parallelEncoder, imu, initialPos, initialHeading);

        waitForStart();

        while (!isStopRequested()) {
            localizer.update();

            telemetry.addData("Position X", localizer.getPos().x);
            telemetry.addData("Position Y", localizer.getPos().y);
            telemetry.addData("Heading", localizer.getAngle());

            telemetry.update();
        }
    }
}
