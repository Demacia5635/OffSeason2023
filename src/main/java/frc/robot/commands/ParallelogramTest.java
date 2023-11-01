// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Parallelogram;

public class ParallelogramTest extends CommandBase {
  public Parallelogram parallelogram;
  public double pow;
  /** Creates a new ParallelogramTest. */
  public ParallelogramTest(Parallelogram parallelogram) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.parallelogram = parallelogram;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pow = SmartDashboard.getNumber("Power", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    parallelogram.setPow(pow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return parallelogram.getInput();
  }
}
