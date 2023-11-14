// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class ArmControl extends CommandBase {

  public Arm arm;
  public CommandXboxController controller;

  /** Creates a new Parrelogram. */
  public ArmControl(Arm arm, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.controller = controller;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setPow(-1*(controller.getLeftY()*0.3));
  }

  // Called once the command ends or is interrupted.
  @Override
public void end(boolean interrupted) { arm.stop(); }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
