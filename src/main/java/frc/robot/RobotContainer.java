package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;


public class RobotContainer {
  CommandXboxController commandController;
  Chassis chassis;
  DriveCommand drive;
  public Arm arm;
  public Gripper gripper;

  public RobotContainer() {

    commandController = new CommandXboxController(Constants.CONTROLLER_PORT);
    chassis = new Chassis();
    drive = new DriveCommand(chassis, commandController);
    arm = new Arm();
    gripper = new Gripper();

    chassis.setDefaultCommand(drive);

    SmartDashboard.putData("set velocity", new RunCommand(() -> chassis.setVelocities(new ChassisSpeeds(1.2, 0.6, Math.toRadians(39)))));
    configureBindings();
  }

    /**
     * 
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // code for controller to controll the gripper and the parallelogram

        // safty buttons to stop the arm and/or the gripper
        commandController.leftBumper().onTrue(new InstantCommand(()-> gripper.stop(), gripper));
        commandController.rightBumper().onTrue(new InstantCommand(()-> arm.stop(), arm));
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand();
  }
}