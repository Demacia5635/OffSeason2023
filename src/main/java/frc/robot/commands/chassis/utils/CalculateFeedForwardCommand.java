package frc.robot.commands.chassis.utils;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;

public class CalculateFeedForwardCommand extends CommandBase {
  public CalculateFeedForwardCommand(Chassis chassis, double p) {
    

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void end(boolean interrupted) {
  }
}
