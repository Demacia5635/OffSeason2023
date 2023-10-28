// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Arc extends Segment{

    //p1 represents the start point, p2 represents the circle center
    Rotation2d angle;


    final Translation2d startVector;
    final double radius;
    /**
     * 
     * @param p1 - Start point of arc
     * @param p2 - Circle center of arc
     * @param angle - Arc's angle
     */
    public Arc(Translation2d p1, Translation2d p2, Rotation2d angle)
    {
        //start point
        this.p1 = p1;
        this.p2 = p2;
        this.angle = angle;

        startVector = p1.minus(p2);
        radius = startVector.getNorm();
    }

    public Translation2d calc(Translation2d pos,double velocity)
    {
        Translation2d relativePos = p2.minus(pos);

        Translation2d fixVector = relativePos.times(-1).div(relativePos.getNorm()).times(relativePos.getNorm() - radius).times(0.5/*kP*/);

        Translation2d velVector = 
        relativePos.rotateBy(
        new Rotation2d(
          Math.toRadians(90 * Math.signum(angle.getDegrees()))
          )
        )
        .div(relativePos.getNorm())
        .times(velocity)
        .plus(fixVector);
      return velVector;
    }

    public double distancePassed(Translation2d pos)
    {
      Translation2d relativePos = p2.minus(pos);

      Rotation2d diffAngle = startVector.getAngle().minus(relativePos.getAngle());
      return diffAngle.getRadians() * radius;
    }



}
