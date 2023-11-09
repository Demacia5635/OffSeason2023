// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Parallelogram;

public class ParallelogramGoToAngle extends CommandBase {
  Parallelogram parallelogram;
  double wantedAngle;
  double pow = 0.15;
  boolean isStart = false;

  /** Creates a new ParallelogramGoToAngle. */
  public ParallelogramGoToAngle(Parallelogram parallelogram, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.parallelogram = parallelogram;
    wantedAngle = angle;
    addRequirements(parallelogram);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (parallelogram.getAngle() > wantedAngle){
      parallelogram.setPow(-pow);
    } else if(parallelogram.getAngle() < wantedAngle){
      parallelogram.setPow(pow);
    }

    if (parallelogram.getAngle()>1){
      isStart = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { parallelogram.stop(); }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (parallelogram.getInput()&&isStart) || (Math.abs(parallelogram.getAngle()-wantedAngle)<1);
  }
}
