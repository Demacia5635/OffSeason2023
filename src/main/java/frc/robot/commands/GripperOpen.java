// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Gripper.GripperState;
import static frc.robot.Constants.GripperConstants.*;

public class GripperOpen extends CommandBase {
    Gripper gripper;

    /** Creates a new GripperOpen. */
    public GripperOpen(Gripper gripper) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.gripper = gripper;
        addRequirements(gripper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        gripper.setPower(openPower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return gripper.getState() == GripperState.OPEN;
    }
}
