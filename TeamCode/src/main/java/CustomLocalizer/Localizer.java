package CustomLocalizer;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Util.Encoder;
import Util.Vector2;
import CustomLocalizer.Constants;

public class Localizer {
    private Encoder perpEncoder;
    private Encoder parallelEncoder;
    private IMU imu;

    private Vector2 pos;
    private int prevParallelTicks;
    private int prevPerpTicks;
    private double prevTheta;
    private double initialTheta;

    public Localizer(Encoder perpEncoder, Encoder parallelEncoder, IMU imu, Vector2 initialPos, double initialTheta) {
        this.perpEncoder = perpEncoder;
        this.parallelEncoder = parallelEncoder;
        this.pos = initialPos;
        this.prevTheta = initialTheta;
        this.imu = imu;

        this.parallelEncoder.setDirection(Encoder.Direction.REVERSE);
        this.perpEncoder.setDirection(Encoder.Direction.REVERSE);

        this.prevParallelTicks = parallelEncoder.getCurrentPosition();
        this.prevPerpTicks = perpEncoder.getCurrentPosition();
        this.initialTheta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        this.prevTheta = initialTheta;
    }

    public void update() {
        Vector2 deltaTicks = new Vector2(perpEncoder.getCurrentPosition() - prevPerpTicks, parallelEncoder.getCurrentPosition() - prevParallelTicks);
        Vector2 deltaInches = deltaTicks.multiply(Constants.TICKS_PER_INCH).multiply(new Vector2(Constants.X_MULTIPLIER, Constants.Y_MULTIPLIER));

        double theta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - initialTheta;
        double averageTheta = (prevTheta + theta) / 2.0;
        double cosTheta = Math.cos(averageTheta);
        double sinTheta = Math.sin(averageTheta);

        Vector2 deltaGlobal = new Vector2(deltaInches.y * cosTheta + deltaInches.x * sinTheta, deltaInches.y * sinTheta - deltaInches.x * cosTheta);
        pos = pos.add(deltaGlobal);

        prevPerpTicks = perpEncoder.getCurrentPosition();
        prevParallelTicks = parallelEncoder.getCurrentPosition();
        prevTheta = theta;
    }

    public Vector2 getPos() {
        Vector2 out = pos.multiply(Math.PI);
        return new Vector2(out.y, out.x);
    }

    public double getAngle() {
        return prevTheta;
    }
}
