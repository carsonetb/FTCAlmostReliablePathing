package Opmodes.Auto.CustomLocalizer;

public class PIDAxis {
    private double P;
    private double I;
    private double D;

    private double lastTime;
    private double lastError;
    private double integralSum;

    public boolean isAngle = false;

    public PIDAxis(double P, double I, double D) {
        this.P = P;
        this.I = I;
        this.D = D;

        this.lastTime = System.nanoTime() / 1e9;
    }

    private static double angleDifferenceRadians(double target, double current) {
        double diff = target - current;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        return diff;
    }

    public double calculate(double target, double current) {
        double currentTime = System.nanoTime() / 1e9;
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        double error = target - current;
        if (isAngle) {
            error = angleDifferenceRadians(target, current);
        }
        integralSum += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        return P * error + I * integralSum + D * derivative;
    }
}
