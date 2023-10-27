// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
abstract class Segment {
    Translation2d p1;
    Translation2d p2;
    
    abstract Translation2d calc(Translation2d position, double velocity);
    abstract double distancePassed(Translation2d position);
}
