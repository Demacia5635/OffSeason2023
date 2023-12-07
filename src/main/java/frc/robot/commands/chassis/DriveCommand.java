package frc.robot.commands.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;

import static frc.robot.Constants.ChassisConstants.*;

public class DriveCommand extends CommandBase {
  private final Chassis chassis;
  private final CommandXboxController controller;

  public DriveCommand(Chassis chassis, CommandXboxController controller) {
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
    double joyX = deadband(controller.getLeftX(), 0.1);
    double joyY = deadband(controller.getLeftY(), 0.1);
    double rot = deadband(controller.getRightTriggerAxis(), 0.1) - deadband(controller.getLeftTriggerAxis(), 0.1);

    ChassisSpeeds speeds = new ChassisSpeeds(joyY * VELOCITY, joyX * VELOCITY, Math.toDegrees(rot * ANGULAR_VELOCITY));
    chassis.setVelocities(speeds);
  }

  private double deadband(double x, double threshold) {
    if (Math.abs(x) < threshold) return 0;
    else return x;
  }
}
