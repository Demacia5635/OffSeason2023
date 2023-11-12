package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;

import static frc.robot.Constants.ChassisConstants.*;

public class Drive extends CommandBase {
  private final Chassis chassis;
  private final CommandXboxController controller;
  
  private boolean precisionDrive;

  public Drive(Chassis chassis, CommandXboxController controller) {
    this.chassis = chassis;
    this.controller = controller;

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    chassis.stop();
    precisionDrive = false;
  }

  @Override
  public void execute() {
      double x = Math.pow(deadband(controller.getLeftX(), 0.1), 3);
      double y = Math.pow(deadband(controller.getLeftY(), 0.1), 3);
      double rot = Math.pow(deadband(controller.getRightTriggerAxis(), 0.1) - deadband(controller.getLeftTriggerAxis(), 0.1), 3);

      double vel = precisionDrive ? VELOCITY / 2 : VELOCITY;
      // TODO: get if the arm is retracted in order to rotate slowly
      ChassisSpeeds speeds = new ChassisSpeeds(y * vel, x * vel, Math.toRadians(rot * ANGULAR_VELOCITY));
      chassis.setVelocities(speeds);

      controller.a().onTrue(new InstantCommand(() -> precisionDrive = !precisionDrive));
  }

  private double deadband(double x, double minDeadband) {
    if (Math.abs(x) < minDeadband) return 0;
    return x;
  }
}
