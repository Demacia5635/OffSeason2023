package frc.robot;

import com.fasterxml.jackson.databind.cfg.PackageVersion;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util.pathPoint;
import frc.robot.commands.ArcPath;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveLine;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(0);
  Chassis chassis = new Chassis();
  DriveLine driveLine = new DriveLine(chassis);
  pathPoint[] points = { 
     new pathPoint(0,0, new Rotation2d() ,0.25),
     new pathPoint(1,1, new Rotation2d() ,0.25),
     new pathPoint(2,0, new Rotation2d(), 0.25),
     new pathPoint(3, 1, new Rotation2d(), 0.25)
    };
     
  ArcPath path = new ArcPath(chassis, points, 1, 2);
  Drive drive = new Drive(chassis, controller);
  public RobotContainer() {
    chassis.setDefaultCommand(drive);

  }
  
  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return driveLine;
  }
}
