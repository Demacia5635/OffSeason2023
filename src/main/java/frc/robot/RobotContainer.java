package frc.robot;

<<<<<<< HEAD


import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.PathFollow;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveLine;
=======
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.chassis.DriveCommand;
>>>>>>> main
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

<<<<<<< HEAD
public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(0);
  Chassis chassis = new Chassis();
  DriveLine driveLine = new DriveLine(chassis);
=======

public class RobotContainer {
  CommandXboxController commandController;
  Chassis chassis;
  DriveCommand drive;
  public Arm arm;
  public Gripper gripper;
>>>>>>> main


  pathPoint[] points = { 
     new pathPoint(0,0, new Rotation2d() ,0.25, true),
     new pathPoint(1,1, new Rotation2d() ,1, true),
     new pathPoint(2,0, new Rotation2d() ,0.25, false),

    };
     
  PathFollow path = new PathFollow(chassis, points, 2, 2);
  Drive drive = new Drive(chassis, controller);
  
  public RobotContainer() {
<<<<<<< HEAD
=======

    commandController = new CommandXboxController(Constants.CONTROLLER_PORT);
    chassis = new Chassis();
    drive = new DriveCommand(chassis, commandController);
    arm = new Arm();
    gripper = new Gripper();

    chassis.setDefaultCommand(drive);

    SmartDashboard.putData("set velocity", new RunCommand(() -> chassis.setVelocities(new ChassisSpeeds(1.2, 0.6, Math.toRadians(39)))));
>>>>>>> main
    configureBindings();
    chassis.setDefaultCommand(drive);

<<<<<<< HEAD
    

    
  }
  private void configureBindings() {
  }
=======
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
>>>>>>> main

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
<<<<<<< HEAD
    return path;
=======
    return new InstantCommand();
>>>>>>> main
  }
}