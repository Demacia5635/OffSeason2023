package frc.robot;

// import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmControl;
import frc.robot.commands.ArmGoToAngle;
import frc.robot.commands.GripperClose;
import frc.robot.commands.GripperControl;
import frc.robot.commands.GripperOpen;
import frc.robot.commands.ArmGoBack;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;


public class RobotContainer {
  public Arm arm = new Arm();
  public Gripper gripper = new Gripper();
  public CommandXboxController controller = new CommandXboxController(0);
  
  public RobotContainer() {
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

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
