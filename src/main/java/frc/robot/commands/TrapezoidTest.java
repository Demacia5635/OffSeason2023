package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.Trapezoid;

public class TrapezoidTest extends CommandBase {
  Chassis chassis;
  Trapezoid t;

  public TrapezoidTest(Chassis chassis) {
    this.chassis = chassis;
    t = new Trapezoid(180, 360);

    addRequirements(chassis);
  }

  @Override
  public void execute() {
    double v = t.calculate(chassis.getDifference(82), chassis.getVel(), 0);
    chassis.setWheelAngles(SmartDashboard.getNumber("angle", 0));
  }
}
