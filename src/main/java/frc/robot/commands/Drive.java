package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;

public class Drive extends CommandBase {
  private final Chassis chassis;
  private final CommandXboxController controller;

  public Drive(Chassis chassis, CommandXboxController controller) {
    this.chassis = chassis;
    this.controller = controller;

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    chassis.stop();
  }

  @Override
  public void execute() {
      double x = deadband(controller.getLeftX(), 0.1);
      double y = deadband(controller.getLeftY(), 0.1);
      double rot = deadband(controller.getRightTriggerAxis(), 0.1) - deadband(controller.getLeftTriggerAxis(), 0.1);

      ChassisSpeeds speeds = new ChassisSpeeds(y * 1.4, x * 1.4, Math.toRadians(rot * 90));
      chassis.setVelocities(speeds);
  }

  private double deadband(double x, double minDeadband) {
    if (Math.abs(x) < minDeadband) return 0;
    return x;
  }
}
