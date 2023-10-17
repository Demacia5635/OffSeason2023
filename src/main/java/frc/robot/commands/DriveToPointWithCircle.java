
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.RoundedPoint;
import frc.robot.Util.Trapezoid;
import frc.robot.subsystems.Chassis;

public class DriveToPointWithCircle extends CommandBase {
  Chassis chassis;
  Translation2d wantedPoint, nextPoint;
  RoundedPoint roundedPoint;
  Translation2d[] points;
  Rotation2d wantedAngle = new Rotation2d();
  Pose2d pose;
  double remainingDistance;
  double maxVelocity = 3;
  double wantedAccel = 6;
  Trapezoid driveTrapezoid = new Trapezoid(wantedAccel, maxVelocity);
 
  

  public DriveToPointWithCircle(Translation2d wantedPoint, Translation2d nextPoint, Rotation2d wantedAngle , Chassis chassis) {
    this.chassis = chassis;
    this.wantedPoint = wantedPoint;
    this.nextPoint = nextPoint;
    this.wantedAngle = wantedAngle;
    roundedPoint = new RoundedPoint(0, chassis.getPose().getTranslation(), wantedPoint, nextPoint);
    points = roundedPoint.getPoints();
  }


  @Override
  public void initialize() {

  }


  @Override
  public void execute() {
    while (chassis.getPose().getTranslation() != points[0]){
      System.out.println("START");
      pose = chassis.getPose();
      Translation2d vector = wantedPoint.minus(pose.getTranslation());
      double angleError = wantedAngle.minus(pose.getRotation()).getDegrees();
      remainingDistance = vector.getNorm();
      double velocity = driveTrapezoid.calculate(remainingDistance, chassis.getVelocity().getNorm(), 0);
      Translation2d driveAngle = vector.div(remainingDistance).times(velocity);
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(3 , 3, Math.toRadians(angleError * 0.03));
      System.out.println("DRIVE");
      chassis.setVelocity(chassisSpeeds);
    }
    

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
