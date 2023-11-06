package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.test;

public class RobotContainer {

  test test = new test();
  public RobotContainer() {

  }
  
  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return test;
  }
}
