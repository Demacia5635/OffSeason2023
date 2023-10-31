
package frc.robot;

import static frc.robot.Constants.*;

import frc.robot.commands.ArcPath;
import frc.robot.commands.Test;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.roundedPath;


public class RobotContainer extends CommandBase{
  
  Chassis chassis = new Chassis();
  
  CommandXboxController driverController = new CommandXboxController(DriverControllerPort);
  //DriveToPointWithCircle driveToPointWithCircle = new DriveToPointWithCircle(new Translation2d(1, 1), new Translation2d(0, 2), new Rotation2d(0), chassis);

  Test test = new Test();


  Translation2d[] points = {new Translation2d(0,0), new Translation2d(-1,1), new Translation2d(-3,-1), new Translation2d(-4,0)};
  double[] radius = {0.1,0.1};
  ArcPath arcPath = new ArcPath(chassis, points, radius, 4, 0.5);





  public RobotContainer() {


    configureBindings();
  }


  private void configureBindings() {

    




  }
  


  public Command getAutonomousCommand() {
    return test;
  }
}
