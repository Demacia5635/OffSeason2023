// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.RoundedPoint;
import frc.robot.Util.Trapezoid;
import frc.robot.subsystems.Chassis;

public class roundedPath extends CommandBase {
  /** Creates a new roundedPath. */

  double pathLength;
  RoundedPoint[] corners;
  Trapezoid trapezoid;
  Chassis chassis;

  //Current corner the robot's on and it's index
  RoundedPoint cCorner;
  double cIndex = 0;
  
  //the distance left for the robot in a segment (a to curve's start,curve,c to curve's end)
  double segLeft;
  //the distance left for the robot in total
  double totalLeft;

  boolean isFinished = false;

  //current & previous positions
  Translation2d cPos = new Translation2d(0,0);
  Translation2d pPos = new Translation2d(0,0);

  public roundedPath(Chassis chassis,Translation2d[] points, double[] radius, double maxVel, double maxAcc) {
    this.corners = new RoundedPoint[points.length - 2];
    for(int i = 0; i < points.length - 2; i++)
    {
      corners[i] = new RoundedPoint(radius[i], points[i], points[i+1], points[i+2]);
    }

    this.chassis = chassis;
    this.trapezoid = new Trapezoid(maxAcc, maxVel);

    this.pathLength = 0;
    for(int i = 0; i < corners.length - 1; i++)
    {
      this.pathLength += corners[i].getAtoCurvelength() + corners[i].getCurveLength();
      System.out.println("added a to curve length : " + corners[i].getAtoCurvelength() + " | added curve length : " + corners[i].getCurveLength());
    }
    this.pathLength += corners[corners.length - 1].getAtoCurvelength() + corners[corners.length - 1].getCurveLength() + corners[corners.length - 1].getCtoCurvelength();
    System.out.println("added a to curve length : " + corners[corners.length - 1].getAtoCurvelength() + " | added curve length : " + corners[corners.length - 1].getCurveLength() + " | added c to curve length : " + corners[corners.length - 1].getCtoCurvelength());
    System.out.println("Path Length : " + this.pathLength);

    totalLeft = this.pathLength;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PathLength : " + pathLength);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vel = trapezoid.calculate(totalLeft, chassis.getVelocity().getNorm(), 0);



    double distance = cPos.minus(pPos).getNorm();
    totalLeft -= distance;
    segLeft -= distance;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
