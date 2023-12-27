// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class Trapez {
    double acc;
    double maxVel;
    double endVel;
    double deltaV;

    public Trapez(double acc, double maxVel, double endVel)
    {
        this.acc = acc;
        this.maxVel = maxVel;
        this.endVel = endVel;
        deltaV = acc * 0.02;
    }

    public double calc(double rDistance, double cVel)
    {
        System.out.println("rDistance: " + rDistance);
        System.out.println("DISTANCE TO SLOW: " + accDistance(cVel));
        //The question is - to accelerate or not accelerate?
        //Check if the distance that's left, is enough for the distance to deaccelerate to the end velocity at an accelerated velocity (cVel + deltaV)
        if(Math.abs(rDistance) > Math.abs(accDistance(cVel)))
        {

            //If yes, strive for the max velocity
            System.out.println("STRIVE FOR MAX");
            System.out.println();
            if(rDistance >= 0)
                return Math.min(maxVel, cVel + (deltaV * Math.signum(rDistance)));
            else
                return Math.max(maxVel * -1, cVel + (deltaV * Math.signum(rDistance)));
        }
        else
        {
            //If no, strive for the min velocity
            System.out.println("STRIVE FOR END");
            if(rDistance >= 0)
                return Math.max(endVel, cVel - (deltaV * Math.signum(rDistance)));
            else
                return Math.min(endVel* -1, cVel - (deltaV * Math.signum(rDistance)));
        }
    }

    /**
     * 
     * @param cVel - Current Velocity (M/S)
     * @return The distance (meters) passed in the next cycle, with acceleration.
     */
    /*private double cycDistance(double cVel)
    {
        double t = maxVel - cVel;
        return Constants.CYCLE_DT * cVel + 0.5 * acc * Math.pow(Constants.CYCLE_DT, 2);
    }*/

    /**
     * 
     * @param cVel - Current Velocity (M/S)
     * @return The distance (meters) required to deaccelerate from current velocity to end velocity (according to the acceleration)
     */
    private double accDistance(double cVel)
    {
        double direction = (Math.signum(cVel) == 0) ? Math.signum(this.maxVel) : Math.signum(cVel);
        return (Math.pow(endVel, 2) - Math.pow(cVel + deltaV * direction, 2)) / (-2 * acc * direction);
    }
}
