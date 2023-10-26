// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Parallelogram;
import static frc.robot.Constants.ParallelogramConstants.*;

public class ParallelogramBackAndForth extends CommandBase {
  public Parallelogram parallelogram;
  public double vel;

  /** Creates a new ParallelogramBackAndForth. */
  public ParallelogramBackAndForth(Parallelogram parallelogram, double vel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.parallelogram = parallelogram;
    this.vel = vel;
    addRequirements(parallelogram);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new ParallelogramGoToAngle(parallelogram,vel,minAngle);
    new ParallelogramGoToAngle(parallelogram,vel,maxAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public double getVel(){ return vel; }
  
  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);
      builder.addDoubleProperty("Velocity", this::getVel, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
