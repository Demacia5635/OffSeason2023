package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;

public class SetWheelAngles extends CommandBase {
  private final Chassis chassis;

  private double angle = 0;

  public SetWheelAngles(Chassis chassis) {
    this.chassis = chassis;

    addRequirements(chassis);

    SmartDashboard.putData(this);
  }

  @Override
  public void initialize() {
    chassis.stop();
  }

  @Override
  public void execute() {
    chassis.setWheelAngles(angle);
  }

  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("angle", null, (x) -> angle = x);
  }
}
