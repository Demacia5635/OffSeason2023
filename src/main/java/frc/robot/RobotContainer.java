package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcPath;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {
  
  Chassis chassis = new Chassis();
  Translation2d pointA = new Translation2d(0,0);
  Translation2d pointB = new Translation2d(1,1);
  Translation2d pointC = new Translation2d(2,0);
  Translation2d pointD = new Translation2d(3,1);
  Translation2d[] points = {pointA, pointB, pointC, pointD};
  double[] radius = {0.1, 0.1, 0.1, 0.1};
  ArcPath pathFollow = new ArcPath(chassis, points, radius, 2, 4);
  public RobotContainer() {


  }
  
  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return pathFollow;
  }
}
