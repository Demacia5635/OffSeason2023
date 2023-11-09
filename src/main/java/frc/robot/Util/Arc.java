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
        super(p1,p2);
        this.angle = angle;

        startVector = p1.minus(p2);
        radius = startVector.getNorm();
        
    }

    @Override
    public Translation2d calc(Translation2d pos,double velocity)
    {
        Translation2d relativePos = pos.minus(p2);
        double dFromCenter = relativePos.getNorm();

        Rotation2d tAngle = new Rotation2d(velocity / radius);



        Translation2d fixVector = relativePos.times(-1).div(relativePos.getNorm()).times(relativePos.getNorm() - radius).times(0.5/*kP*/);

        Rotation2d tanAngle = relativePos.getAngle().minus(new Rotation2d(Math.toRadians(90 * Math.signum(angle.getDegrees()))));
        Rotation2d fixAngle = tAngle.times(dFromCenter / radius);


      
        
      return new Translation2d(velocity, tanAngle.plus(fixAngle));
    }

    @Override
    public double distancePassed(Translation2d pos)
    {
      Translation2d relativePos = p2.minus(pos);

      Rotation2d diffAngle = startVector.getAngle().minus(relativePos.getAngle());
      return Math.abs(diffAngle.getRadians() * radius);
    }

    @Override
    public double getLength()
    {
      return Math.abs(angle.getRadians()) * radius;
    }

    @Override
    public String toString() {
        return "\n~Arc~\nStartPoint : " + p1 + "\nCircleCenter : " + p2 + "\nAngle : " + angle + "\nRadius : " + radius;
    }

}
