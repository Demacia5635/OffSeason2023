// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmControl;
import frc.robot.commands.ArmGoToAngle;
import frc.robot.commands.ArmGoBack;
import frc.robot.commands.ArmStateCaculator;
import frc.robot.subsystems.Arm;


public class RobotContainer {
  public Arm arm = new Arm();
  public CommandXboxController controller = new CommandXboxController(0);
  
  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    controller.a().onTrue(new ArmControl(arm, controller));
    controller.b().onTrue(new ArmGoBack(arm));
    controller.x().onTrue(new ArmGoToAngle(arm, 50));
    // run the caculator that get the state the arm, and run also a command that move the arm to 70 and then back to 0 degrees
    controller.y().onTrue(new ParallelCommandGroup(new ArmGoToAngle(arm, 70).andThen(new ArmGoBack(arm)), new ArmStateCaculator(arm)));

    // in case start to end does not work
    // controller.y().onTrue(new ArmSetAngle(arm).andThen(new ArmGoToAngle(arm, 70).andThen(new ArmSetAngle(arm))));

    controller.rightTrigger().onTrue(new InstantCommand(()-> arm.stop(),arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new ArmGoBack(arm);
  }
}
