package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
  XboxController controller = new XboxController(0);
  CommandXboxController commandController = new CommandXboxController(0);
  Chassis chassis = new Chassis();
  Drive drive;
  SetWheelAngles angles;
  LedController ledController = new LedController(Constants.LedConstants.ID, Constants.LedConstants.LED_COUNT);

  public RobotContainer() {
    drive = new Drive(chassis, commandController);
    angles = new SetWheelAngles(chassis);
    chassis.setDefaultCommand(drive);
    
    SmartDashboard.putData("reset wheels", new InstantCommand(() -> chassis.setWheelAngles(0)));
    SmartDashboard.putData("set coast", new InstantCommand(() -> chassis.setNeutralMode(NeutralMode.Coast)));
    SmartDashboard.putData("set brake", new InstantCommand(() -> chassis.setNeutralMode(NeutralMode.Brake)));


    ledController.changeColor(new Color(29, 0, 51));
    configureBindings();
  }

  private void configureBindings() {
    commandController.y().onTrue(new InstantCommand(() -> ledController.toggleEnabled()));
    commandController.pov(90).whileTrue(new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble, 1)));
    commandController.pov(90).whileFalse(new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble, 0)));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new FunctionalCommand(() -> chassis.setDrivePower(0.2), null, null, null, chassis);
    return angles;
    // return null;
  }
}
