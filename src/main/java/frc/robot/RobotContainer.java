package frc.robot;



import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.PathFollow;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveLine;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(0);
  Chassis chassis = new Chassis();
  DriveLine driveLine = new DriveLine(chassis);


  pathPoint[] points = { 
     new pathPoint(0,0, new Rotation2d() ,0.25, true),
     new pathPoint(1,1, new Rotation2d() ,1, true),
     new pathPoint(2,0, new Rotation2d() ,0.25, false),

    };
     
  PathFollow path = new PathFollow(chassis, points, 2, 2);
  Drive drive = new Drive(chassis, controller);
  
  public RobotContainer() {
    configureBindings();
    chassis.setDefaultCommand(drive);

    

    
  }
  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return path;
  }
}