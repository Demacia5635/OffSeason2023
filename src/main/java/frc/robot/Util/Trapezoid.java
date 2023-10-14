package frc.robot.Util;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;


public class Trapezoid {
    Chassis chassis;
    private double deltaV;

    private double MaxAccel;
    private double MaxVelocity;



    public Trapezoid(double MaxAccel, double MaxVelocity){
        this.MaxAccel = MaxAccel;
        this.MaxVelocity = MaxVelocity;
        deltaV = MaxAccel * Constants.cycleTime;
    }



    public double calculate(double distanceLeft, double CurrentVelocity, double endV){


        if (accelDistance() > distanceLeft) {
            return CurrentVelocity - deltaV;
        } 
        else if(CurrentVelocity >= MaxVelocity){
            return MaxVelocity;
            
        } else{
            return CurrentVelocity + deltaV;
        }
    }

    private double accelDistance(){
        double t = MaxVelocity / MaxAccel;
        double d1 = (0.5 * MaxAccel * t * t);
            
    return d1;
            
    }

}


