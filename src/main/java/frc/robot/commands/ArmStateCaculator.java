// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmStateCaculator extends CommandBase {
  public Arm arm;
  public double startAngle = 0;
  public double endAngle = 70;
  public double switchAngle = 40;
  public int state;
  boolean isStart = false;
  boolean isForward;

  /** Creates a new ArmBackAndForth. */
  public ArmStateCaculator(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
    SmartDashboard.putData(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isForward = true;
    state = 4;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!(Math.abs(endAngle-arm.getAngle())<2) && isForward){
      if (startAngle <= arm.getAngle() && arm.getAngle() < switchAngle){
          state = 0;
      } else if (endAngle > arm.getAngle() && arm.getAngle() >= switchAngle){
          state = 1;
      }
    } else {
      isForward = false;
      if (endAngle >= arm.getAngle() && arm.getAngle() > switchAngle){
          state = 2;
      } else if (startAngle < arm.getAngle() && arm.getAngle() <= switchAngle){
          state = 3;
      }
    }

    if (arm.getAngle()>1){
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
      builder.addIntegerProperty("state", ()->{return state;}, null);

      SmartDashboard.putNumber("Start Angle", startAngle);
      SmartDashboard.putNumber("End Angle", endAngle);
      SmartDashboard.putNumber("Switch Angle", switchAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.getInput()&&isStart) || state == 4 || (((Math.abs(arm.getAngle()))<0.5)&&isStart);
  }
}
