package frc.robot.Util;
import frc.robot.Constants;


public class Trapezoid {
    private double deltaV;

    private double MaxAccel;
    private double MaxVelocity;



    public Trapezoid(double MaxAccel, double MaxVelocity){
        System.out.println("ENTERED");
        this.MaxAccel = MaxAccel;
        this.MaxVelocity = MaxVelocity;
        deltaV = MaxAccel * Constants.CYCLE_DT;

    }



    public double calculate(double distanceLeft, double CurrentVelocity, double endV){


        if (accelDistance() > distanceLeft) {
            System.out.println("DEACCEL: " + Math.min(CurrentVelocity - deltaV, endV));
            return Math.min(CurrentVelocity - deltaV, endV);
        } 
        else if(CurrentVelocity >= MaxVelocity){
            System.out.println("KEEP");
            return MaxVelocity;
            
        } else{
            System.out.println("ACCEL: " + Math.max(CurrentVelocity + deltaV, MaxVelocity));
            return Math.max(CurrentVelocity + deltaV, endV);
        }
    }

    private double accelDistance(){
        double t = MaxVelocity / MaxAccel;
        double d1 = (0.5 * MaxAccel * t * t);
        System.out.println("DISTANCE: " + d1);  
        System.out.println("DELTA V: " + deltaV);  
        return d1;
            
    }

}


