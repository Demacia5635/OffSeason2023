// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.util.ArmCalc;

public class ArmGoToAngle extends CommandBase {
  Arm arm;
  double wantedAngle;
  ArmCalc calc;
  static final double maxVel = 90;
  static final double minVel = 0;
  static final double acc = 120;

  /** Creates a new ArmGoToAngle. */
  public ArmGoToAngle(Arm arm, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    wantedAngle = angle;
    addRequirements(arm);
    calc = new ArmCalc(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vel = calc.trapezoid(arm.getCurrentAnglerVel(), maxVel,minVel, acc, wantedAngle-arm.getAngle());
    arm.setVel(vel);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { arm.stop(); }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.getInput()&&wantedAngle == 0) || (Math.abs(arm.getAngle()-wantedAngle)<3);
  }
}
