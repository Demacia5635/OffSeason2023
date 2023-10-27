// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ArcDrive extends CommandBase {
  /** Creates a new ArcDrive. */
  Chassis chassis;
  Translation2d sPoint;
  Translation2d cCenter;
  Rotation2d angle;
  double maxVel;

  final double radius;

  public ArcDrive(Chassis chassis,Translation2d sPoint ,Translation2d cCenter, Rotation2d angle,double maxVel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.sPoint = sPoint;
    this.cCenter = cCenter;
    this.angle = angle;
    this.maxVel = maxVel;

    this.radius = cCenter.minus(sPoint).getNorm();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
