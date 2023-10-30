// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Segment {
    protected Translation2d p1;
    protected Translation2d p2;
    
    public Segment(Translation2d p1, Translation2d p2)
    {
        this.p1 = p1;
        this.p2 = p2;
    }

    public Translation2d calc(Translation2d position, double velocity) {return new Translation2d();};
    public double distancePassed(Translation2d position) {return 0;};
    public double getLength() {return 0;};
}
