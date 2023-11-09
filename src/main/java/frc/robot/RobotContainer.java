package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcPath;
import frc.robot.commands.test;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {

  test test = new test();
  Chassis chassis = new Chassis();
  Translation2d[] points = { new Translation2d(0,0), new Translation2d(1,1),new Translation2d(3,-1), new Translation2d(5,1)};
  double[] radius = {0.5,0.5,0.5,0.5};
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
