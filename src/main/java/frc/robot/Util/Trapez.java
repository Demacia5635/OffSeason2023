// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import frc.robot.Constants;

/** Add your docs here. */
public class Trapez {
    double acc;
    double maxVel;
    double endVel;

    public Trapez(double acc, double maxVel, double endVel)
    {
        this.acc = acc;
        this.maxVel = maxVel;
        this.endVel = endVel;
    }

    public double calc(double rDistance, double cVel)
    {
        //The question is - to accelerate or not accelerate?
        //Check if the distance that's left after acceleration, is enough for the distance to deaccelerate to the end velocity 
        if(rDistance - cycDistance(cVel) >= accDistance(cVel))
        {
            //If yes, strive for the max velocity
            return Math.min(maxVel, cVel + acc);
        }
        else
        {
            //If no, strive for the min velocity
            return Math.max(endVel, cVel - acc);
        }
    }

    /**
     * 
     * @param cVel - Current Velocity (M/S)
     * @return The distance (meters) passed in the next cycle, with acceleration.
     */
    private double cycDistance(double cVel)
    {
        return Constants.CYCLE_DT * cVel + 0.5 * acc * Math.pow(Constants.CYCLE_DT, 2);
    }

    /**
     * 
     * @param cVel - Current Velocity (M/S)
     * @return The distance (meters) required to deaccelerate from current velocity to end velocity (according to the acceleration)
     */
    private double accDistance(double cVel)
    {
        return (Math.pow(cVel + acc, 2) + Math.pow(endVel, 2)) / (-2 * acc);
    }
}
