// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.Trapezoid;
import frc.robot.subsystems.chassis.Chassis;

public class DriveLine extends CommandBase {
  Translation2d point = new Translation2d(0, 4);
  Chassis chassis;
  Trapezoid trap = new Trapezoid(2, 1, 0, 0);
  public DriveLine(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }


  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d velocity = new Translation2d(0, trap.calculate(point.getY() - chassis.getPoseY(), chassis.getVelocity()));
    System.out.println("CALC: " + velocity.getY());
    ChassisSpeeds speed = new ChassisSpeeds(0,velocity.getY(), 0);
    chassis.setVelocities(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.getPose().getEstimatedPosition().getTranslation().getNorm() >= point.getNorm();
  }
}
