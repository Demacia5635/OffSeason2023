package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.chassis.Chassis;

public class WheelAngles extends InstantCommand {
  private final Chassis chassis;

  public WheelAngles(Chassis chassis) {
    this.chassis = chassis;
    
    addRequirements(chassis);
    SmartDashboard.putNumber("angle", 0);
  }

  @Override
  public void initialize() {
    double angle = SmartDashboard.getNumber("angle", 0);
    chassis.setWheelAngles(angle);
  }
}
