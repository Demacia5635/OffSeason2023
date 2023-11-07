// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Parallelogram;

public class ParallelogramControl extends CommandBase {

  public Parallelogram parallelogram;
  public CommandXboxController controller;

  /** Creates a new Parrelogram. */
  public ParallelogramControl(Parallelogram parallelogram, CommandXboxController controller) {
    this.parallelogram = parallelogram;
    this.controller = controller;
    addRequirements(parallelogram);
    SmartDashboard.putData(this);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    parallelogram.setPow(-1*(controller.getLeftY()*0.3));
  }

  // Called once the command ends or is interrupted.
  @Override
public void end(boolean interrupted) { parallelogram.stop(); }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
