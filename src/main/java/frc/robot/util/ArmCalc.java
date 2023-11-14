// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.subsystems.Arm;

/** Add your docs here. */
public class ArmCalc {
    Arm arm;
    
    public ArmCalc(Arm arm){
        this.arm = arm;
    }

    public double trapezoid(double maxVel, double minVel, double acc, double dis){
        double d1 = (maxVel-minVel)/acc;
        double d3 = dis - d1;

        if (arm.getAngle() < d1){
            if (arm.getCurrentAnglerVel()+acc >= maxVel){
                return maxVel;
            } else {
                return arm.getCurrentAnglerVel()+acc;
            }
        } else if (arm.getAngle() > d3){
            if (arm.getCurrentAnglerVel()-acc <= minVel){
                return minVel;
            } else {
                return arm.getCurrentAnglerVel()-acc;
            }
        } else {
            return arm.getCurrentAnglerVel();
        }
    }
}
