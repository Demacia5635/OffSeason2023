package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SongCommand;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.utils.CalculateFF;
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
    return new CalculateFF(chassis);

    // return new RunCommand(()->chassis.setModulesPower(0.12), chassis);

  // return new RunCommand(()->chassis.setModulesAngularVelocity(100), chassis)
  // //  .withTimeout(1)
  //   .andThen( 
  //   new InstantCommand(()->
  //   {SmartDashboard.putNumber("modules velocities 0", chassis.getAngularVelocities()[0]);
  //   SmartDashboard.putNumber("modules velocities 1", chassis.getAngularVelocities()[1]);
  //   SmartDashboard.putNumber("modules velocities 2", chassis.getAngularVelocities()[2]);
  //   SmartDashboard.putNumber("modules velocities 3", chassis.getAngularVelocities()[3]);
  // chassis.stop();}, chassis));
  }
}
