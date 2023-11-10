// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ParallelogramControl;
import frc.robot.commands.ParallelogramGoToAngle;
import frc.robot.commands.ParallelogramSetAngle;
import frc.robot.commands.ParallelogramStartToEnd;
import frc.robot.subsystems.Parallelogram;


public class RobotContainer {
  public Parallelogram parallelogram = new Parallelogram();
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
    controller.a().onTrue(new ParallelogramControl(parallelogram, controller));
    controller.b().onTrue(new ParallelogramSetAngle(parallelogram));
    controller.x().onTrue(new ParallelogramGoToAngle(parallelogram, 50));
    controller.y().onTrue(new ParallelogramStartToEnd(parallelogram));

    // in case start to end does not work
    // controller.y().onTrue(new ParallelogramSetAngle(parallelogram).andThen(new ParallelogramGoToAngle(parallelogram, 70).andThen(new ParallelogramSetAngle(parallelogram))));

    controller.rightTrigger().onTrue(new InstantCommand(()-> parallelogram.stop(),parallelogram));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new ParallelogramSetAngle(parallelogram);
  }
}
