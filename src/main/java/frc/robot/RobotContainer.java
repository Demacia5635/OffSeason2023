package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;

import static frc.robot.Constants.LedConstants.*;

public class RobotContainer {
  CommandXboxController commandController = new CommandXboxController(Constants.CONTROLLER_PORT);
  Chassis chassis = new Chassis();

  public RobotContainer() {
    /*`
    SmartDashboard.putData("find drive ff", new RunMotorsCommand(chassis,
      Constants.ChassisConstants.PULSES_PER_METER,
      -0.2,
      new int[] {1, 3, 5, 7}
    ).withTimeout(2)
    .andThen(
      new RunMotorsCommand(chassis,
        Constants.ChassisConstants.PULSES_PER_METER,
        -0.4,
        new int[] {1, 3, 5, 7}
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
  */

    SmartDashboard.putNumber("v", 0);
    SmartDashboard.putData("drive", new RunCommand(() -> chassis.setVelocities(new ChassisSpeeds(SmartDashboard.getNumber("v", 0), 0, 0))));

  LedController c = new LedController(LED_ID, LED_COUNT);
  c.stop();

    configureBindings();
  }

  private void configureBindings() {
 
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
