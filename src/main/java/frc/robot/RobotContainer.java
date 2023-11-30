package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveSimple;
import frc.robot.commands.WheelAngles;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {
  XboxController controller = new XboxController(0);
  CommandXboxController commandController = new CommandXboxController(0);
  Chassis chassis = new Chassis();
  Drive drive;
  LedController ledController = new LedController(Constants.LedConstants.ID, Constants.LedConstants.LED_COUNT);

  WheelAngles angles;

  public RobotContainer() {
    drive = new Drive(chassis, commandController);
    chassis.setDefaultCommand(new DriveSimple(commandController, chassis));
    
    SmartDashboard.putData("set coast", new InstantCommand(() -> chassis.setNeutralMode(NeutralMode.Coast)).ignoringDisable(true));
    SmartDashboard.putData("set brake", new InstantCommand(() -> chassis.setNeutralMode(NeutralMode.Brake)).ignoringDisable(true));

    SmartDashboard.putNumber("v", 0);
    SmartDashboard.putData("set wheel angles test command", new RunCommand(() -> chassis.setWheelAngularVelocities(SmartDashboard.getNumber("v", 0))));

    ledController.changeColor(new Color(29, 0, 51));
    ledController.stop();

    configureBindings();
  }

  private void configureBindings() {
    commandController.pov(90).whileTrue(new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble, 1)).ignoringDisable(true));
    commandController.pov(90).whileFalse(new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble, 0)).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
