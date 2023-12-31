package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.utils.SwerveModule;

public class RobotContainer {
  CommandXboxController commandController = new CommandXboxController(Constants.CONTROLLER_PORT);
  Chassis chassis = new Chassis();
  DriveCommand drive = new DriveCommand(chassis, commandController);

  public RobotContainer() {
    chassis.setDefaultCommand(drive);

    // SmartDashboard.putData("set velocity", new RunCommand(() -> chassis.setVelocities(new ChassisSpeeds(1.2, 0.6, Math.toRadians(39)))));
    SmartDashboard.putData("set module velocity",
     new RunCommand(() -> chassis.getModule(0).setAngularVelocity(40))
     .alongWith(new RunCommand(() -> chassis.getModule(1).setAngle(Rotation2d.fromDegrees(70))))
     .alongWith(new RunCommand(() -> chassis.getModule(2).setAngularVelocity(180)))
    //  .alongWith(new RunCommand(() -> chassis.getModule(3).setAngularVelocity(40), chassis))
     .withTimeout(2).andThen(
      new InstantCommand(()->System.out.println(chassis.getModule(1).getAngularVelocity())),
      new InstantCommand(() -> chassis.getModule(0).setAngularVelocity(0), chassis),
      new InstantCommand(() -> chassis.getModule(1).setAngularVelocity(0), chassis),
      new InstantCommand(() -> chassis.getModule(2).setAngularVelocity(0), chassis)
      // new InstantCommand(() -> chassis.getModule(3).setAngularVelocity(0), chassis)
      ));

    configureBindings();
  }

  private void configureBindings() {
 
  }

  public Command getAutonomousCommand() {

    // return new RunCommand(()->chassis.setModulesPower(0.12), chassis);

    return new InstantCommand(()->chassis.setModulesPower(0.6), chassis)
    .andThen(new WaitCommand(0.25), 
    new InstantCommand(()->
    {SmartDashboard.putNumber("modules velocities 0", chassis.getVelocities()[0]);
    SmartDashboard.putNumber("modules velocities 1", chassis.getVelocities()[1]);
    SmartDashboard.putNumber("modules velocities 2", chassis.getVelocities()[2]);
    SmartDashboard.putNumber("modules velocities 3", chassis.getVelocities()[3]);}, chassis));
  }
}
