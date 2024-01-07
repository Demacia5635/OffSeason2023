package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.SetWheelAngles;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);
  Chassis chassis = new Chassis();
  DriveCommand drive;
  SetWheelAngles angles;

  public RobotContainer() {
    drive = new DriveCommand(chassis, controller);
    angles = new SetWheelAngles(chassis);
    chassis.setDefaultCommand(drive);
    
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new FunctionalCommand(() -> chassis.setDrivePower(0.2), null, null, null, chassis);
    return angles;
    // return null;
  }
}