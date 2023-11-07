// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.subsystems.Parallelogram;

/** Add your docs here. */
public class ParallelogramCaculator {
    public Parallelogram parallelogram;
    public double angle;
    private boolean isForward = true;

    public ParallelogramCaculator(Parallelogram parallelogram){
        this.parallelogram = parallelogram;
        angle = parallelogram.getAngle();
    }

    public int ParallelogramStartToEnd(double startAngle, double endAngle,double switchAngle, double pow){
        if (!(endAngle <= angle) && isForward){
            if ((startAngle < angle && angle < switchAngle)|| angle <= startAngle){
                return 0;
            } else if ((endAngle > angle && angle > switchAngle)|| angle <= switchAngle){
                return 1;
            }
        } else {
            isForward = false;
            if ((endAngle > angle && angle > switchAngle)|| angle >= endAngle){
                return 2;
            } else if ((startAngle < angle && angle < switchAngle)|| angle <= switchAngle){
                return 3;
            }
        }
        return 4;
    }
}
