// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PathFollow.Util.Trapez;
import frc.robot.subsystems.chassis.Chassis;

public class DriveLine extends CommandBase {
  Translation2d point = new Translation2d(0, 4);
  Chassis chassis;
  Translation2d velocity = new Translation2d();
  Trapez trap = new Trapez(1, 0.5, 0);
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

    ChassisSpeeds speed = new ChassisSpeeds(0,1, 0.5);
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
    return chassis.getPose().getTranslation().getNorm() >= point.getNorm();
  }
}
