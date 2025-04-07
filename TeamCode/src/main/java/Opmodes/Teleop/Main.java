package Opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import Components.Arm;
import Components.ButtonHandler;
import Components.Drive;
import Components.Hang;
import Components.Slides;
import Util.Vector2;

@TeleOp(name = "Main", group = "Working")
public class Main extends OpMode {

    IMU imu;
    Drive drive;
    Hang hang;
    Slides slides;
    Arm arm;
    double servo_pos = 0;

    ButtonHandler clawButtonHandler;

    @Override
    public void init() {
        // Imu config
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        // Drive config
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");

        drive = new Drive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, imu);

        // Slides config
        DcMotor slideMotor = hardwareMap.dcMotor.get("slidesMotor");

        slides = new Slides(slideMotor);

        // Hang config
        DcMotor leftHangMotor = hardwareMap.dcMotor.get("leftHangMotor");
        DcMotor rightHangMotor = hardwareMap.dcMotor.get("parallelEncoder");

        hang = new Hang(leftHangMotor, rightHangMotor);

        // Arm config
        Servo armServo = hardwareMap.servo.get("armServo");
        Servo clawServo = hardwareMap.servo.get("clawServo");

        arm = new Arm(armServo, clawServo);
        clawButtonHandler = new ButtonHandler();
    }

    @Override
    public void loop() {
        // Drive
        if (!gamepad2.y) {
            Vector2 driveDirection = new Vector2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            float driveRotation = gamepad1.right_stick_x;
            drive.moveInDirection(driveDirection, driveRotation, 1.0f);
        }

        Vector2 preciseDirection = new Vector2(-gamepad2.left_stick_y, gamepad2.left_stick_x);
        float preciseRotation = gamepad2.right_stick_x * 0.6f;
        drive.moveInDirection(preciseDirection, preciseRotation, 0.35f);

        // Reset rotation
        if (gamepad1.options) {
            imu.resetYaw();
        }

        // Slides
        float slidesPower = (gamepad2.right_trigger - gamepad2.left_trigger);
        slides.moveSlides(slidesPower * 100.0);

        // Hang
        if (gamepad2.dpad_up) {
            hang.goUp(1);
        }
        else if (gamepad2.dpad_down) {
            hang.goDown(1);
        }
        else {
            hang.stop();
        }

        clawButtonHandler.update(gamepad2.a);

        if (clawButtonHandler.justPressed) {
            if (arm.clawState == Arm.CLAW_STATE.OPEN) {
                arm.closeClaw();
            }
            else if (arm.clawState == Arm.CLAW_STATE.CLOSED) {
                arm.openClaw();
            }
        }

        if (gamepad2.left_bumper) {
            arm.moveArmUp();
        }
        if (gamepad2.right_bumper) {
            arm.moveArmDown();
        }

        arm.setArmPosition(arm.armServo.getPosition() - gamepad2.right_stick_y * 0.005);
        arm.debugPosition(telemetry);

        telemetry.addData("Is above high bar", slides.isAboveHighBar());

        telemetry.addData("Slides: Position", slides.slideMotor.getCurrentPosition());
        telemetry.addData("Supposed servo pos", servo_pos);

        telemetry.update();
    }
}