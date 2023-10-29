// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.Arc;
import frc.robot.Util.Leg;
import frc.robot.Util.RoundedPoint;
import frc.robot.Util.Trapezoid;
import frc.robot.subsystems.Chassis;

public class ArcPath extends CommandBase {

  Chassis chassis;
  RoundedPoint[] corners;


  double pathLength;
  double totalLeft;
  int segmentIndex = 0;
  int segmentPosition = 0;

  //Segment[] segments;

  Leg[] legs;
  Arc[] arcs;
  Trapezoid trapezoid;
  /** Creates a new ArcPath.
   * @param chassis 
   * @param point Translation2d array of points for the path
   * @param radius double array of radius (for each turn)
   * @param maxVel the max velocity in m/s
   * @param maxAccel the max accel in m/s2 (squared)
   * 
   */

  public ArcPath(Chassis chassis,Translation2d[] points, double[] radius, double maxVel, double maxAcc) {

    this.corners = new RoundedPoint[points.length - 2];
    for(int i = 0; i < points.length - 2; i++)
    {
      corners[i] = new RoundedPoint(radius[i], points[i], points[i+1], points[i+2]);
    }
    this.chassis = chassis;
    addRequirements(chassis);

    trapezoid = new Trapezoid(maxAcc, maxVel);

    //calculate the total length of the path
    pathLength = 0;
    for(int i = 0; i < corners.length - 1; i++)
    {
      pathLength += corners[i].getAtoCurvelength() + corners[i].getCurveLength();
      System.out.println("added a to curve length : " + corners[i].getAtoCurvelength() + " | added curve length : " + corners[i].getCurveLength());
    }
    this.pathLength += corners[corners.length - 1].getAtoCurvelength() + corners[corners.length - 1].getCurveLength() + corners[corners.length - 1].getCtoCurvelength();
    System.out.println("added a to curve length : " + corners[corners.length - 1].getAtoCurvelength() + " | added curve length : " + corners[corners.length - 1].getCurveLength() + " | added c to curve length : " + corners[corners.length - 1].getCtoCurvelength());
    System.out.println("Path Length : " + this.pathLength);

    totalLeft = this.pathLength;

    //segments = new Segment[points.length - 1];
    arcs = new Arc[corners.length];
    legs = new Leg[corners.length + 1];

    for(int i = 0; i < corners.length; i++)
    {

      arcs[i] = corners[i].getArc();
      legs[i] = corners[i].getAtoCurveLeg();
    }

    legs[legs.length - 1] = corners[corners.length - 1].getCtoCurveLeg();


    
  }




  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double velocity = trapezoid.calculate(totalLeft , chassis.getVelocity().getNorm(), 0);
    Translation2d vecVel = new Translation2d(0,0);
    totalLeft -= legs[segmentIndex].getLength();


    if((segmentPosition+ 1) % 2 == 0)
    {
      vecVel = legs[segmentIndex].calc(chassis.getPose().getTranslation(), velocity);
      if(legs[segmentIndex].distancePassed(chassis.getPose().getTranslation()) >= legs[segmentIndex].getLength())
      {
        segmentPosition++;
      }
    }
    else
    {
      vecVel = arcs[segmentIndex].calc(chassis.getPose().getTranslation(), velocity);
      if(arcs[segmentIndex].distancePassed(chassis.getPose().getTranslation()) >= arcs[segmentIndex].getLength())

      segmentIndex++;
      segmentPosition++;
    }
    

    ChassisSpeeds speed = new ChassisSpeeds(0,0,0);
    chassis.setVelocity(speed);
  }


  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
