package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.SetWheelAngles;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {
  CommandXboxController controller;
  Chassis chassis;
  DriveCommand drive;

  public RobotContainer() {
    chassis = new Chassis();
    drive = new DriveCommand(chassis, controller);
    chassis.setDefaultCommand(drive);
    
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new FunctionalCommand(() -> chassis.setDrivePower(0.2), null, null, null, chassis);
    return null;
    // return null;
  }
}