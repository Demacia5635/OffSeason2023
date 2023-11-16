package frc.robot.Util;

import java.rmi.AccessException;

import frc.robot.Constants;

public class Trapezoid {
    private final double maxVelocity;
    private final double acceleration;
    private final double safeVelocity;
    private final double endVelocity;
    double deltaV;
    boolean firstTime;

    public Trapezoid(double maxVelocity, double acceleration, double safeVelocity, double endVelocity) {
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.safeVelocity = safeVelocity;
        this.endVelocity = endVelocity;
        deltaV = acceleration * Constants.CYCLE_DT;
        firstTime = true;
    }

    public double calculate(double remainingDistance, double currentVelocity) {
        if (requiredDistance(currentVelocity + deltaV, 0, -acceleration) <= remainingDistance ){
            System.out.println("ACCEL: + " + Math.min(maxVelocity, currentVelocity + deltaV));
            return Math.min(maxVelocity, currentVelocity + deltaV);
        }
       else if(requiredDistance(currentVelocity, 0, -acceleration) <= remainingDistance){
            System.out.println("KEEP: " + currentVelocity);
            return currentVelocity;
            
       }
       else{
            System.out.println("DEACCEL: " + Math.max(currentVelocity-deltaV, endVelocity));
            return Math.max(currentVelocity-deltaV, endVelocity);
       }
    
    }

    public static double requiredDistance(double fromVelocity, double toVelocity, double acceleration) {
        double time = (toVelocity - fromVelocity)/acceleration;
        System.out.println("TIME: " + time);
        System.out.println("ACCEL DISTANCE: " + time*fromVelocity + (acceleration*time*time)/2);
        return time*fromVelocity + (acceleration*time*time)/2;
    }

}
