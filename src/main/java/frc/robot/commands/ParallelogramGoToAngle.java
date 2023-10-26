// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Parallelogram;

public class ParallelogramGoToAngle extends CommandBase {
  public Parallelogram parallelogram;
  public double pow;
  public double wantedAngle;
  /** Creates a new ParrellelogramBackAndForth. */
  public ParallelogramGoToAngle(Parallelogram parallelogram, double pow, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.parallelogram = parallelogram;
    this.pow = pow;
    this.wantedAngle = angle;
    addRequirements(parallelogram);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (wantedAngle > parallelogram.getAngle()){
      parallelogram.setPow(pow);
    } else if (wantedAngle < parallelogram.getAngle()){
      parallelogram.setPow(-pow);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    parallelogram.stop();
  }

  public double getPow(){ return pow; }
  
  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);
      builder.addDoubleProperty("Power", this::getPow, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(parallelogram.getAngle()==wantedAngle){
      return true;
    }
    return false;
  }
}