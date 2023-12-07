// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;




import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.Segment;
import frc.robot.Util.Trapez;
import frc.robot.Constants;
import frc.robot.Util.Leg;
import frc.robot.Util.RoundedPoint;
import frc.robot.Util.pathPoint;
import frc.robot.subsystems.chassis.*;

public class ArcPath extends CommandBase {
  Pose2d closestAprilTag = new Pose2d();

  Chassis chassis;
  RoundedPoint[] corners;
  Pose2d pose = new Pose2d();

  double distanceOffset = 0.01;

  final double pathLength;

  double totalLeft;
  int segmentIndex = 0;

  Segment[] segments;
  Translation2d vecVel;
  Rotation2d wantedAngle;


  Trapez driveTrapezoid;
  Trapez rotationTrapezoid;
  double velocity = 0;
  double rotationVelocity = 0;
  double safeVel = 1;

  /** Creates a new ArcPath.
   * @param chassis 
   * @param point Translation2d array of points for the path
   * @param radius double array of radius (for each turn)
   * @param maxVel the max velocity in m/s
   * @param maxAccel the max accel in m/s2 (squared)
   * 
   */

  public ArcPath(Chassis chassis,pathPoint[] points, double maxVel, double maxAcc) {

    wantedAngle = points[points.length - 1].getRotation();
    corners = new RoundedPoint[points.length - 2];
    for(int i = 0; i < points.length - 2; i++)
    {
      corners[i] = new RoundedPoint(points[i], points[i+1], points[i+2]);
    }
    this.chassis = chassis;
    addRequirements(chassis);

    

    SmartDashboard.putData(this);

    driveTrapezoid = new Trapez(maxAcc, maxVel, 0);
    rotationTrapezoid = new Trapez(180, 180, 0);

    //calculate the total length of the path

    segments = new Segment[1 + ((points.length - 2) * 2)];

   

    segments[0] = corners[0].getAtoCurveLeg();
    int segmentIndexCreator = 1;
    for(int i = 0; i < corners.length - 1; i +=1)
    {
      segments[segmentIndexCreator] = corners[i].getArc(); 
      segments[segmentIndexCreator+1] = new Leg(corners[i].getCurveEnd(), corners[i+1].getCurveStart(), false);
      segmentIndexCreator+=2;
    }
    segments[segments.length - 2] = corners[corners.length - 1].getArc();
    segments[segments.length - 1] = corners[corners.length - 1].getCtoCurveLeg();

    System.out.println("Segment length : " + segments.length);

    double segmentSum = 0;
    for (Segment s : segments) {
      segmentSum += s.getLength();
    }
    pathLength = segmentSum;
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

    totalLeft = pathLength;

    segmentIndex = 0;

    vecVel = new Translation2d(0,0);  
  }

  public Pose2d getClosestAprilTag(){
    Translation2d finalVector = new Translation2d();
    int finalAprilTagIndex = 0;
    for(int i = 0; i < 8; i++){
      
      Translation2d currentAprilTagVector =chassis.getPose().getEstimatedPosition().getTranslation()
      .minus(Constants.aprilTagsPositions[i].getTranslation());

     if(currentAprilTagVector.getNorm() < finalVector.getNorm()){
      finalAprilTagIndex++;
      finalVector = currentAprilTagVector;
     }
      
    }
    System.out.println("CLOSET APRILTAG: " +  new Pose2d(finalVector, Constants.aprilTagsPositions[finalAprilTagIndex].getRotation()));
    return new Pose2d(finalVector, Constants.aprilTagsPositions[finalAprilTagIndex].getRotation());
  }


  @Override
  public void execute() {
    pose = chassis.getPose().getEstimatedPosition();
    Translation2d translation2dVelocity = new Translation2d(chassis.getVelocity().vxMetersPerSecond, chassis.getVelocity().vyMetersPerSecond);

    
    if(segments[segmentIndex].distancePassed(pose.getTranslation()) >= segments[segmentIndex].getLength() - distanceOffset){
      totalLeft -= segments[segmentIndex].getLength();
      if(segmentIndex != segments.length - 1 || segments[segmentIndex].getLength() <= 0.15)
        segmentIndex++;  
    }

    velocity = driveTrapezoid.calc(totalLeft - segments[segmentIndex].distancePassed(pose.getTranslation()), translation2dVelocity.getNorm());

    if(!segments[segmentIndex].isAprilTagMode())
      rotationVelocity = rotationTrapezoid.calc(wantedAngle.minus(chassis.getAngle()).getDegrees(), Math.toDegrees(chassis.getVelocity().omegaRadiansPerSecond));
    else
      rotationVelocity = rotationTrapezoid.calc(
        getClosestAprilTag().getTranslation().minus(chassis.getPose().getEstimatedPosition().getTranslation())
        .getAngle()
          .minus(
            chassis.getAngle()).getDegrees()
        ,  Math.toDegrees(chassis.getVelocity().omegaRadiansPerSecond));

    Translation2d velVector = segments[segmentIndex].calc(pose.getTranslation(), velocity);
    ChassisSpeeds speed = new ChassisSpeeds();

    //TODO ADD Rotation to closet AprilTag
    speed = new ChassisSpeeds(velVector.getX(), velVector.getY(),Math.toRadians(rotationVelocity));
    
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
