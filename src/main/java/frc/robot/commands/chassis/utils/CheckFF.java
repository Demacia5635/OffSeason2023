package frc.robot.commands.chassis.utils;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;

import static frc.robot.Constants.ChassisConstants.SwerveModuleConstants.*;

public class CheckFF extends CommandBase {
  private final double v = 1.1;
  private final Chassis chassis;

  private double timer = 0;

  public CheckFF(Chassis chassis) {
    this.chassis = chassis;

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
      chassis.resetWheels();
      chassis.setModulesPower(MOVE_KS * Math.signum(v) + MOVE_KV * v);
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
    return timer >= 2;
  }
}
