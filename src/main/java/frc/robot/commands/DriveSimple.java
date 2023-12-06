package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;

public class DriveSimple extends CommandBase {
  private final CommandXboxController controller;
  private final Chassis chassis;

  public DriveSimple(CommandXboxController controller, Chassis chassis) {
    this.controller = controller;
    this.chassis = chassis;

    addRequirements(chassis);
  }

  @Override
    public void execute() {
      chassis.setWheelAngularVelocities(deadband(controller.getLeftY(), 0.1) * 180);
    }

    private double deadband(double x, double minDeadband) {
      if (Math.abs(x) < minDeadband) return 0;
      return x;
    }
}
