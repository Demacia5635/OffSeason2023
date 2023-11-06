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
      double x = controller.getLeftX();
      double y = controller.getLeftY();
      double rot = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();

      ChassisSpeeds speeds = new ChassisSpeeds(y, x, rot);
      chassis.setVelocities(speeds);
  }
}
