package frc.robot;

import java.time.Instant;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.PathFollow;
import frc.robot.commands.SongCommand;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.utils.CheckFF;
import frc.robot.commands.DriveLine;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(0);
  Chassis chassis = new Chassis();


  pathPoint[] points = { 
     new pathPoint(0,0, new Rotation2d() ,0.1, false),
     new pathPoint(1,1, new Rotation2d() ,0.1, false),
     new pathPoint(02,0, new Rotation2d() ,0.1, false),
      new pathPoint(3,1, new Rotation2d() ,0.1, false),

    };

    
     
  PathFollow path = new PathFollow(chassis, points, 0.1, 1);
  DriveCommand drive = new DriveCommand(chassis, controller);
  DriveLine driveLine = new DriveLine(chassis);
  
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