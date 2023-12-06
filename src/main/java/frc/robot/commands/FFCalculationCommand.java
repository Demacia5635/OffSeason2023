package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;

public class FFCalculationCommand extends CommandBase {
  Chassis chassis;
  double p;

  public FFCalculationCommand(Chassis chassis, double p) {
    this.chassis = chassis;
    this.p = p;

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
      chassis.setWheelsPower(p);
  }

  @Override
  public void end(boolean interrupted) {
      chassis.stop();
      chassis.printVelocities();
  }
}
