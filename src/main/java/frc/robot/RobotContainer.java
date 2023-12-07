package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    SmartDashboard.putNumber("angle", 0);
    SmartDashboard.putData("set angles", new RunCommand(() -> {
      SwerveModuleState[] states = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        states[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(SmartDashboard.getNumber("angle", 0)));
      }
      chassis.setModuleStates(states);
    }, chassis));

    configureBindings();
  }

  private void configureBindings() {
 
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
