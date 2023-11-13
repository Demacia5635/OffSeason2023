package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcPath;
import frc.robot.commands.test;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {

  test test = new test();
  Chassis chassis = new Chassis();
  Translation2d[] points = { new Translation2d(0,0), new Translation2d(4,4),new Translation2d(8,0),new Translation2d(12,4)};
  double[] radius = {2,2};
  ArcPath path = new ArcPath(chassis, points, radius, 0, 0);
  public RobotContainer() {

  }
  
  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return test;
  }
}
