// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Parallelogram;
import frc.robot.util.ParallelogramCaculator;

public class ParallelogramStartToEnd extends CommandBase {
  public Parallelogram parallelogram;
  public double startAngle = 0;
  public double endAngle = 70;
  public double switchAngle = 40;
  public double pow = 0.15;
  public ParallelogramCaculator caculator;
  public int state;
  boolean isStart = false;

  /** Creates a new ParallelogramBackAndForth. */
  public ParallelogramStartToEnd(Parallelogram parallelogram) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.parallelogram = parallelogram;
    addRequirements(parallelogram);
    SmartDashboard.putData(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    caculator = new ParallelogramCaculator(parallelogram);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    state = caculator.parallelogramStartToEnd(startAngle, endAngle, switchAngle, pow);
    if (parallelogram.getAngle()>1){
      isStart = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { parallelogram.stop(); }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addIntegerProperty("state", ()->{return state;}, null);

      SmartDashboard.putNumber("Start Angle", startAngle);
      SmartDashboard.putNumber("End Angle", endAngle);
      SmartDashboard.putNumber("Switch Angle", switchAngle);
      SmartDashboard.putNumber("Wanted Power", pow);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (parallelogram.getInput()&&isStart) || state == 4 || (((Math.abs(parallelogram.getAngle()))<0.5)&&isStart);
  }
}
