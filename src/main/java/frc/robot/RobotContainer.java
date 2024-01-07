package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive;
import frc.robot.commands.SetWheelAngles;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);
  Chassis chassis = new Chassis();
  Drive drive;
  SetWheelAngles angles;

  public RobotContainer() {
    drive = new Drive(chassis, controller);
    angles = new SetWheelAngles(chassis);
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