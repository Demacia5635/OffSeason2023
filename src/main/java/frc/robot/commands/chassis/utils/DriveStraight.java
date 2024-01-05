package frc.robot.commands.chassis.utils;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;

public class DriveStraight extends CommandBase {
  private final double v = 1;
  private final Chassis chassis;

  private double timer = 0;

  public DriveStraight(Chassis chassis) {
    this.chassis = chassis;
  }

  @Override
  public void initialize() {
      timer = 0;
      chassis.resetWheels();
      chassis.setVelocities(new ChassisSpeeds(v, 0, 0));
  }

  @Override
  public void execute() {
      timer += 0.02;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println(chassis.getVelocity().getNorm());
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
      return timer >= 0.02;
  }
}
