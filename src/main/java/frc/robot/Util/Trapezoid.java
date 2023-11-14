package frc.robot.Util;

public class Trapezoid {
    private final double maxVelocity;
    private final double acceleration;
    private final double safeVelocity;
    private final double endVelocity;

    public Trapezoid(double maxVelocity, double acceleration, double safeVelocity, double endVelocity) {
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.safeVelocity = safeVelocity;
        this.endVelocity = endVelocity;
    }

    public double calculate(double remainingDistance, double currentVelocity) {
        double accelTime = maxVelocity / acceleration;
        double accelDistance = 0.5 * acceleration * Math.pow(accelTime, 2);
        if (currentVelocity < maxVelocity && remainingDistance > accelDistance) {
            // accelerate
            return currentVelocity + acceleration * 0.02;
        }
        else if (remainingDistance > accelDistance) {
            // keep velocity
            return maxVelocity;
        }
        else if (remainingDistance <= accelDistance) {
            // deccelerate
            return Math.max(Math.max(currentVelocity - acceleration * 0.02, safeVelocity), endVelocity);
        }
        return endVelocity;
    }
}
