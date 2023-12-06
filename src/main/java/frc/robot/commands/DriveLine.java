// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.Trapez;
import frc.robot.subsystems.chassis.Chassis;

public class DriveLine extends CommandBase {
  Translation2d point = new Translation2d(0, 4);
  Chassis chassis;
  Translation2d velocity = new Translation2d();
  Trapez trap = new Trapez(2, 4, 0);
  public DriveLine(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("VELOCITY", () -> {return velocity.getY();},null);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocity = new Translation2d(0, trap.calc(point.getY() - chassis.getPoseY(), chassis.getSpeeds().vyMetersPerSecond));
    System.out.println("CALC: " + trap.calc(point.getY() - chassis.getPoseY(), chassis.getSpeeds().vyMetersPerSecond));
    System.out.println("VELOCITY: " + velocity.getY());
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
