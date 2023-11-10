package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive;
import frc.robot.commands.SetWheelAngles;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);
  Chassis chassis = new Chassis();
  Drive drive;
  SetWheelAngles angles;
  LedController ledController = new LedController(Constants.LedConstants.ID, Constants.LedConstants.LED_COUNT);

  public RobotContainer() {
    drive = new Drive(chassis, controller);
    angles = new SetWheelAngles(chassis);
    chassis.setDefaultCommand(drive);
    
    ledController.changeColor(new Color(29, 0, 51));
    configureBindings();
  }

  private void configureBindings() {
    controller.a().onTrue(new InstantCommand(() -> ledController.toggleEnabled()));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new FunctionalCommand(() -> chassis.setDrivePower(0.2), null, null, null, chassis);
    return angles;
    // return null;
  }
}
