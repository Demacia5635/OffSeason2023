// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.Segment;
import frc.robot.Util.Arc;
import frc.robot.Util.Leg;
import frc.robot.Util.RoundedPoint;
import frc.robot.Util.Trapezoid;
import frc.robot.subsystems.chassis.*;

public class ArcPath extends CommandBase {

  Chassis chassis;
  RoundedPoint[] corners;
  Pose2d pose = new Pose2d();

  double distanceOffset = 5;

  

  double pathLength;
  double totalLeft;
  int segmentIndex = 0;

  Segment[] segments;
  Translation2d vecVel;

  Trapezoid trapezoid;
  double velocity = 0;

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
    for(int i = 0; i < corners.length; i++)
    {
      pathLength += corners[i].getAtoCurvelength() + corners[i].getCurveLength();
      System.out.println("added a to curve length : " + corners[i].getAtoCurvelength() + " | added curve length : " + corners[i].getCurveLength());
    }
    pathLength =+ corners[corners.length - 1].getCtoCurvelength();
    System.out.println("added a to curve length : " + corners[corners.length - 1].getAtoCurvelength() + " | added curve length : " + corners[corners.length - 1].getCurveLength() + " | added c to curve length : " + corners[corners.length - 1].getCtoCurvelength());
    System.out.println("Path Length : " + this.pathLength);

    totalLeft = pathLength;

    segments = new Segment[points.length + 1];
    


    for(int i = 0,j = 0; i < corners.length; i +=1, j+=2)
    {
      segments[j] = corners[i].getAtoCurveLeg();
      segments[j+1] = corners[i].getArc();
    }

    System.out.println(segments.length);
    segments[segments.length - 1] = corners[corners.length - 1].getCtoCurveLeg();

    for (Segment s : segments) {
      System.out.println(s);
    }
    //segments[0] = new Leg(null, null);
  }




  @Override
  public void initialize() {

    vecVel = new Translation2d(0,0);  
  }



  @Override
  public void execute() {
    pose = chassis.getPose().getEstimatedPosition();

    
    if(segments[segmentIndex].distancePassed(pose.getTranslation()) >= segments[segmentIndex].getLength() - distanceOffset){
      totalLeft -= segments[segmentIndex].getLength();
      segmentIndex++;
    }


    

    
    velocity = trapezoid.calculate(totalLeft - segments[segmentIndex].distancePassed(pose.getTranslation()), chassis.getVelocity().getNorm(), 0);
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
<<<<<<< HEAD
    return false;
=======
    return totalLeft <= 0;
>>>>>>> 752ffd533429aa7ca7065d0714115b3e67bc398d
  }
}
