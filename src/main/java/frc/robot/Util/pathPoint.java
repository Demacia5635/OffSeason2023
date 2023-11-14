// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class pathPoint extends Pose2d{
    public double radius;

    public pathPoint(double x, double y, Rotation2d rotation, double radius) {
        super(x,y,rotation);
        this.radius = radius;
      }

}
