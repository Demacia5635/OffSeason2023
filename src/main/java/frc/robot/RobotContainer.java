package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcPath;
import frc.robot.commands.Drive;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(0);
  Chassis chassis = new Chassis();
  Translation2d[] points = { new Translation2d(0,0), new Translation2d(1,1),new Translation2d(2,0),new Translation2d(3,1)};
  double[] radius = {0.25,0.25};
  ArcPath path = new ArcPath(chassis, points, radius, 1, 2);
  Drive drive = new Drive(chassis, controller);
  public RobotContainer() {
    chassis.setDefaultCommand(drive);

  }
  
  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return path;
  }
}
