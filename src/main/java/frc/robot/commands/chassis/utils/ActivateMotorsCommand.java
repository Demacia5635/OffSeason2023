package frc.robot.commands.chassis.utils;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ActivateMotorsCommand extends CommandBase {
    private final int[] motorIds;

    public ActivateMotorsCommand(Subsystem requirement, int... motors) {
        motorIds = motors;
        addRequirements(requirement);
    }
}