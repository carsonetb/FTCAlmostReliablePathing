package CustomLocalizer;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Util.Encoder;
import Util.Vector2;
import Util.Pose2D;

public class PIDFFollower {
    DcMotorEx leftFrontMotor;
    DcMotorEx leftRearMotor;
    DcMotorEx rightRearMotor;
    DcMotorEx rightFrontMotor;

    Encoder perpendicularEncoder;
    Encoder parallelEncoder;

    Drive mecanumDrive;
    Localizer localizer;

    IMU imu;

    PIDAxis xPID;
    PIDAxis yPID;
    PIDAxis headingPID;

    Telemetry telemetry;

    public PIDFFollower(HardwareMap hardwareMap, Vector2 initialPos, double initialHeading, Telemetry telemetry) {
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

        mecanumDrive = new Drive(leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor, imu);
        localizer = new Localizer(perpendicularEncoder, parallelEncoder, imu, initialPos, initialHeading);

        xPID = new PIDAxis(Constants.P, Constants.I, Constants.D);
        yPID = new PIDAxis(Constants.P, Constants.I, Constants.D);
        headingPID = new PIDAxis(Constants.headingP, Constants.headingI, Constants.headingD);
        headingPID.isAngle = true;

        this.telemetry = telemetry;
    }

    public void updatePIDF(Vector2 pos, Vector2 vel, Vector2 accel, double headingPos, double headingVel, double headingAccel, double customkV, double customkA, double customHeadingkV) {
        Vector2 realPos = localizer.getPos();
        double realHeading = localizer.getAngle();

        Pose2D feedforward = calculateFeedforward(vel, accel, headingVel, headingAccel, customkV, customkA, customHeadingkV);
        double adjustedHeading = headingPos;
        while (adjustedHeading < 0) {
            adjustedHeading += Math.toRadians(360);
        }
        while (adjustedHeading > 360) {
            adjustedHeading -= Math.toRadians(360);
        }
        double adjustedRealHeading = -realHeading;
        if (adjustedRealHeading < 0) {
            adjustedRealHeading = Math.toRadians(360) + adjustedRealHeading;
        }

        double error = adjustedHeading - adjustedRealHeading;
        error = (error + Math.toRadians(180)) % Math.toRadians(360) - Math.toRadians(180);
        telemetry.addData("target", Math.toDegrees(adjustedRealHeading));
        telemetry.addData("real heading", Math.toDegrees(realHeading));
        telemetry.addData("what error should be", Math.toDegrees(error));
        Pose2D totalOffset = new Pose2D(
                feedforward.x - xPID.calculate(pos.x, realPos.x),
                feedforward.y + yPID.calculate(pos.y, realPos.y),
                feedforward.heading + headingPID.calculate(adjustedHeading, adjustedRealHeading)
        );
        setMotorPowers(Drive.poseToMotorPower(totalOffset, realHeading));
    }

    public void updatePIDF(Vector2 pos, Vector2 vel, Vector2 accel, double headingPos, double headingVel, double headingAccel) {
        updatePIDF(pos, vel, accel, headingPos, headingVel, headingAccel, Constants.kV, Constants.kA, Constants.headingkV);
    }

    public void updateFeedforwardOnly(Vector2 vel, Vector2 accel, double headingVel, double headingAccel, double customkV, double customkA, double customHeadingkV) {
        setMotorPowers(Drive.poseToMotorPower(calculateFeedforward(vel, accel, headingVel, headingAccel, customkV, customkA, customHeadingkV), 0));
    }

    public void updateFeedforwardOnly(Vector2 vel, Vector2 accel, double headingVel, double headingAccel) {
        updateFeedforwardOnly(vel, accel, headingVel, headingAccel, Constants.kV, Constants.kA, Constants.headingkV);
    }

    public Pose2D calculateFeedforward(Vector2 vel, Vector2 accel, double headingVel, double headingAccel, double customkV, double customkA, double customHeadingkV) {
        double xFF = calculateFeedforwardAxis(vel.x, accel.x, customkV, customkA);
        double yFF = calculateFeedforwardAxis(vel.y, accel.y, customkV, customkA);
        double headingFF = calculateFeedforwardRotation(headingVel, headingAccel, customHeadingkV, 0);
        return new Pose2D(xFF, yFF, headingFF);
    }

    private double calculateFeedforwardAxis(double vel, double accel, double kV, double kA) {
        return vel * kV + accel * kA;
    }

    private double calculateFeedforwardRotation(double vel, double accel, double kV, double kA) {
        return vel * kV + accel * kA;
    }

    private void setMotorPowers(double[] powers) {
        leftFrontMotor.setPower(powers[0]);
        leftRearMotor.setPower(powers[1]);
        rightFrontMotor.setPower(powers[2]);
        rightRearMotor.setPower(powers[3]);
    }

    public Localizer getLocalizer() { return localizer; }
}
