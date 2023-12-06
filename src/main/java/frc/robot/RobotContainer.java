package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.chassis.RunMotorsCommand;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer {
  CommandXboxController commandController = new CommandXboxController(Constants.CONTROLLER_PORT);
  Chassis chassis = new Chassis();

  public RobotContainer() {
    SmartDashboard.putData("find drive ff", new RunMotorsCommand(chassis,
      Constants.ChassisConstants.PULSES_PER_METER,
      0.2,
      new int[] {1, 3, 5}
    ).withTimeout(2)
    .andThen(
      new RunMotorsCommand(chassis,
        Constants.ChassisConstants.PULSES_PER_METER,
        0.4,
        new int[] {1, 3, 5}
      ).withTimeout(2)
    )
  );

  SmartDashboard.putData("find steer ff", new RunMotorsCommand(chassis,
      Constants.ChassisConstants.PULSES_PER_METER,
      0.2,
      new int[] {2, 4, 6}
    ).withTimeout(2)
    .andThen(
      new RunMotorsCommand(chassis,
        Constants.ChassisConstants.PULSES_PER_METER,
        0.4,
        new int[] {2, 4, 6}
      ).withTimeout(2)
    )
  );

    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
