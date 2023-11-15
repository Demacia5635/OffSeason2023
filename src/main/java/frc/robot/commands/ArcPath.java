// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import java.io.Console;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.Segment;
import frc.robot.Constants;
import frc.robot.Util.Arc;
import frc.robot.Util.Leg;
import frc.robot.Util.RoundedPoint;
import frc.robot.Util.Trapezoid;
import frc.robot.subsystems.chassis.*;

public class ArcPath extends CommandBase {

  Chassis chassis;
  RoundedPoint[] corners;
  Pose2d pose = new Pose2d();

  double distanceOffset = 0.01;

  double pathLength;
  double totalLeft;
  int segmentIndex = 0;

  Segment[] segments;
  Translation2d vecVel;

  Trapezoid trapezoid;
  Trapezoid testTrap;
  double velocity = 0;
  double safeVel = 1;

  /** Creates a new ArcPath.
   * @param chassis 
   * @param point Translation2d array of points for the path
   * @param radius double array of radius (for each turn)
   * @param maxVel the max velocity in m/s
   * @param maxAccel the max accel in m/s2 (squared)
   * 
   */

  public ArcPath(Chassis chassis,Translation2d[] points, double[] radius, double maxVel, double maxAcc) {


    corners = new RoundedPoint[points.length - 2];
    for(int i = 0; i < points.length - 2; i++)
    {
      corners[i] = new RoundedPoint(radius[i], points[i], points[i+1], points[i+2]);
    }
    this.chassis = chassis;
    addRequirements(chassis);

    

    SmartDashboard.putData(this);

    trapezoid = new Trapezoid(maxAcc, maxVel, safeVel, 0);

    //calculate the total length of the path

    segments = new Segment[1 + ((points.length - 2) * 2)];

   

    segments[0] = corners[0].getAtoCurveLeg();
    int segmentIndexCreator = 1;
    for(int i = 0; i < corners.length - 1; i +=1)
    {
      segments[segmentIndexCreator] = corners[i].getArc(); 
      segments[segmentIndexCreator+1] = new Leg(corners[i].getCurveEnd(), corners[i+1].getCurveStart());
      segmentIndexCreator+=2;
    }
    segments[segments.length - 2] = corners[corners.length - 1].getArc();
    segments[segments.length - 1] = corners[corners.length - 1].getCtoCurveLeg();

    System.out.println("Segment length : " + segments.length);

    for (Segment s : segments) {
      pathLength += s.getLength();
    }
    totalLeft = pathLength;

    
    System.out.println("Segments : \n");
    printSegments();

    //segments[0] = new Leg(null, null);

    System.out.println("Velocity calc test : \n");
    System.out.println("Position : " + corners[0].getCurveStart());
    System.out.println(segments[1].calc(corners[0].getCurveStart(),1));
  }


  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addStringProperty("Type",() -> currentSegmentInfo(), null);
      builder.addDoubleProperty("totalLeft",() -> {return totalLeft;}, null);
      builder.addDoubleProperty("Velocity", () -> {return velocity;}, null);
  }


  public String currentSegmentInfo()
  {
    if(segments == null)
      return "";
    return segments[segmentIndex].toString();
  }

  @Override
  public void initialize() {
    segmentIndex = 0;

    vecVel = new Translation2d(0,0);  
  }



  @Override
  public void execute() {
    pose = chassis.getPose().getEstimatedPosition();
    Translation2d translation2dVelocity = new Translation2d(chassis.getVelocity().vxMetersPerSecond, chassis.getVelocity().vyMetersPerSecond);

    
    if(segments[segmentIndex].distancePassed(pose.getTranslation()) >= segments[segmentIndex].getLength() - distanceOffset){
      totalLeft -= segments[segmentIndex].getLength();
      if(segmentIndex != segments.length - 1)
        segmentIndex++;
        
    }


    

    
    velocity = trapezoid.calculate(totalLeft - segments[segmentIndex].distancePassed(pose.getTranslation()), translation2dVelocity.getNorm());
    System.out.println("TRAP: " + velocity);
    Translation2d velVector = segments[segmentIndex].calc(pose.getTranslation(), velocity);
    ChassisSpeeds speed = new ChassisSpeeds(velVector.getX(), velVector.getY(), 0);
    chassis.setVelocities(speed);
  }


  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return totalLeft <= 0;
  }

  public void printSegments()
  {
    for (Segment s : segments) {
      System.out.println(s);
    }
  }
}
