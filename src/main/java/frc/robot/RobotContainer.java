package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {
  CommandXboxController commandController = new CommandXboxController(Constants.CONTROLLER_PORT);
  Chassis chassis = new Chassis();
  DriveCommand drive = new DriveCommand(chassis, commandController);

  public RobotContainer() {
    chassis.setDefaultCommand(drive);

    SmartDashboard.putData("set velocity", new RunCommand(() -> chassis.setVelocities(new ChassisSpeeds(1.2, 0.6, Math.toRadians(39)))));

    configureBindings();
  }

  private void configureBindings() {
 
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
