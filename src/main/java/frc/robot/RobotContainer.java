package frc.robot;

import java.time.Instant;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SongCommand;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.utils.CheckFF;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {
  CommandXboxController commandController = new CommandXboxController(Constants.CONTROLLER_PORT);
  Chassis chassis = new Chassis();
  DriveCommand drive = new DriveCommand(chassis, commandController);

  public RobotContainer() {
    chassis.setDefaultCommand(drive);

    SmartDashboard.putData("sing", new SongCommand());

    configureBindings();
  }

  private void configureBindings() {
 
  }

  public Command getAutonomousCommand() {
    return new RunCommand(()-> chassis.setModulesAngleFromSB(90), chassis);
    // return new InstantCommand(() -> chassis.resetWheels(), chassis)
    // .andThen(new RunCommand(() -> chassis.setVelocities(new ChassisSpeeds(-2, 0, 0))).withTimeout(2).andThen(new InstantCommand(() -> chassis.stop())));
  }
}
