// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.util.ArmCalc;

public class ArmGoToAngle extends CommandBase {
  Arm arm;
  double wantedAngle;
  double pow = 0.15;
  boolean isStart = false;
  ArmCalc calc;
  double maxVel;
  double minVel;
  double acc;
  double dis;

  /** Creates a new ArmGoToAngle. */
  public ArmGoToAngle(Arm arm, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    wantedAngle = angle;
    addRequirements(arm);
    calc = new ArmCalc(arm);
    SmartDashboard.putData(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isStart = false;
    maxVel = 0;
    minVel = 0;
    acc = 0;
    dis = wantedAngle-arm.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setVel(calc.trapezoid(maxVel, minVel, acc, dis));

    if (!arm.getInput()){
      isStart = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { arm.stop(); }

  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);
      SmartDashboard.putNumber("max vel", maxVel);
      SmartDashboard.putNumber("min vel", minVel);
      SmartDashboard.putNumber("acc", acc);
      SmartDashboard.putNumber("dis", dis);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.getInput()&&isStart) || (Math.abs(arm.getAngle()-wantedAngle)<1);
  }
}
