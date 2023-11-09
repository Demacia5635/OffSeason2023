// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Parallelogram;

public class ParallelogramSetAngle extends CommandBase {
  public Parallelogram parallelogram;
  public double pow = 0.15;

  /** Creates a new ParallelogramSetAngle. */
  public ParallelogramSetAngle(Parallelogram parallelogram) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.parallelogram = parallelogram;
    addRequirements(parallelogram);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // parallelogram.setPow(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    parallelogram.setPow(-pow);
    parallelogram.baseAngle = parallelogram.motor.getSelectedSensorPosition()/Constants.ParallelogramConstants.pulsePerAngle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    parallelogram.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return parallelogram.getInput();
  }
}
