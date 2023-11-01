package frc.robot.Util;
import frc.robot.Constants;


public class Trapezoid {
    private double deltaV;

    private double MaxAccel;
    private double MaxVelocity;



    public Trapezoid(double MaxAccel, double MaxVelocity){
        this.MaxAccel = MaxAccel;
        this.MaxVelocity = MaxVelocity;
        deltaV = MaxAccel * Constants.CYCLE_DT;
    }



    public double calculate(double distanceLeft, double CurrentVelocity, double endV){


        if (accelDistance() > distanceLeft) {
            return Math.max(CurrentVelocity - deltaV, endV);
        } 
        else if(CurrentVelocity >= MaxVelocity){
            return MaxVelocity;
            
        } else{
            return Math.min(CurrentVelocity + deltaV, endV);
        }
    }

    private double accelDistance(){
        double t = MaxVelocity / MaxAccel;
        double d1 = (0.5 * MaxAccel * t * t);
            
    return d1;
            
    }

}


