// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.Trapezoid;
import frc.robot.subsystems.Chassis;

public class ArcDrive extends CommandBase {
  /** Creates a new ArcDrive. */
  Chassis chassis;
  Translation2d sPoint;
  Translation2d cCenter;
  Rotation2d angle;
  double maxVel;

  double distanceLeft;
  Translation2d prevTrans;

  final double radius;
  final Translation2d startVector;
  Trapezoid trapezoid;

  public ArcDrive(Chassis chassis,Translation2d sPoint ,Translation2d cCenter, Rotation2d angle,double maxVel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.sPoint = sPoint;
    this.cCenter = cCenter;
    this.angle = angle;
    this.maxVel = maxVel;

    this.trapezoid = new Trapezoid(maxVel, 0.1);

    //addRequirements(chassis);

    this.radius = cCenter.minus(sPoint).getNorm();
    this.startVector = cCenter.minus(sPoint);

    this.distanceLeft = angle.getRadians() * this.radius;
    this.prevTrans = new Translation2d(0,0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d relativePos = chassis.getPose().getTranslation().minus(cCenter);
    double distancePassed = chassis.getPose().getTranslation().minus(prevTrans).getNorm();
    this.distanceLeft -= distancePassed;


    Rotation2d diffAngle = relativePos.getAngle().minus(this.startVector.getAngle());

    //Assuming that the velocity does not change at the end of the arc, it stays on maxVel
    double velocity = trapezoid.calculate(distanceLeft, chassis.getVelocity().getNorm(), maxVel);

    //if the chassis is still on the range of the arc
    if(Math.abs(this.angle.minus(diffAngle).getDegrees()) > 0) {
      Translation2d velVector = 
      relativePos.rotateBy(
        new Rotation2d(
          Math.toRadians(90 * Math.signum(diffAngle.getDegrees()))
          )
        )
        .div(relativePos.getNorm())
        .times(velocity);

      chassis.setVelocity(new ChassisSpeeds(velVector.getX(),velVector.getY(),0));
    }

    this.prevTrans = chassis.getPose().getTranslation();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override 
  public String toString() {
      // TODO Auto-generated method stub
      return "sPoint : " + this.sPoint + "\n" + 
      "cCenter : " + this.cCenter + "\n"+
      "Angle : " + this.angle + "\n"+
      "Max Velocity : " + this.maxVel + "\n" + 
      "Radius : " + this.radius + "\n" +
      "Distance Left : " + this.distanceLeft;
  }
}
