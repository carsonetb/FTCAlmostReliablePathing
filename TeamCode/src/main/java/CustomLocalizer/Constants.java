package CustomLocalizer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class Constants {
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // Encoder ticks per full wheel revolution.
    public static double TICKS_PER_REV = 2000;

    // The radius of the drivetrain wheels (half the diameter!)
    public static double WHEEL_RADIUS = 0.629921; // in

    // Generated, the circumference of the drivetrain wheels, aka the distance all the way around
    // the edge
    public static double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;

    // The amount of ticks required for the dead wheels to travel one inch.
    public static double TICKS_PER_INCH = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;

    // Multipliers in case something is off.
    public static double X_MULTIPLIER = 1;
    public static double Y_MULTIPLIER = 1;

    // The max velocity the robot can travel, in inches per second.
    public static double maxVel = 20;

    // The max acceleration the robot can do, in inches per second squared. Essentially, this is
    // how much the velocity increases every second.
    public static double maxAccel = 20;

    // The max velocity the robot can turn, in degrees per second.
    public static double maxAngVel = 60;

    // The max heading acceleration the robot can do.
    public static double maxAngAccel = 90;


    // Multiplier to convert inches per second to motor power, this is automatically tuned.
    public static double kV = 0.02;

    // Multiplier to convert acceleration to motor power, to help with accuracy. I'm not sure
    // how necessary this is but it's here for compatibility with RoadRunner. Unfortunately I think
    // it's too fine of a variable to calculate automatically.
    public static double kA = 0.00;

    // Multiplier to convert degrees per second to motor power, this is automatically tuned.
    public static double headingkV = 0.316;

    // PID coefficients.
    public static double P = 0.1;
    public static double I = 0;
    public static double D = 0;
    public static double headingP = 2;
    public static double headingI = 0;
    public static double headingD = 0;

    // Max velocity for tuning.
    public static double testMaxVel = 20;

    // Not used currently.
    public static double testMaxAccel = 10;

    // Max rotational velocity for tuning.
    public static double testMaxAngVel = 90;

    // From 0 to 1, lower is better. <0.05 recommended. This is used to calculate the length of
    // the bezier curve.
    public static double bezierPrecision = 0.01;
}
