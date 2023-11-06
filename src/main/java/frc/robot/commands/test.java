// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.Leg;
import frc.robot.Util.RoundedPoint;

public class test extends CommandBase {
  /** Creates a new test. */
  Translation2d aPoint = new Translation2d(0,0);
  Translation2d bPoint = new Translation2d(1,1);
  Translation2d cPoint = new Translation2d(2,0);
  RoundedPoint rPoint = new RoundedPoint(0.1, aPoint, bPoint, cPoint);
  public test() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}