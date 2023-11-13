// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmGoToAngle extends CommandBase {
  Arm arm;
  double wantedAngle;
  double pow = 0.15;
  boolean isStart = false;

  /** Creates a new ArmGoToAngle. */
  public ArmGoToAngle(Arm arm, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    wantedAngle = angle;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {isStart = false;}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.getAngle() > wantedAngle){
      arm.setPow(-pow);
    } else if(arm.getAngle() < wantedAngle){
      arm.setPow(pow);
    }

    if (!arm.getInput()){
      isStart = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { arm.stop(); }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.getInput()&&isStart) || (Math.abs(arm.getAngle()-wantedAngle)<1);
  }
}
