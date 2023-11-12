package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GripperConstants.*;

public class Gripper extends SubsystemBase {
  private final TalonSRX motor;

  public Gripper() {
    motor = new TalonSRX(MOTOR_ID);
  }

  public boolean isClosed() {
    return motor.isFwdLimitSwitchClosed() == 1;
  }

  public boolean isOpened() {
    return motor.isRevLimitSwitchClosed() == 1;
  }

  public void setPower(double p) {
    motor.set(ControlMode.PercentOutput, p);
  }

  public void stop() {
    setPower(0);
  }

  public void open() {
    new FunctionalCommand(null, () -> setPower(OPEN_POWER), (interrupt) -> setPower(0), () -> isOpened(), this);
  }

  public void close() {
    new FunctionalCommand(null, () -> setPower(CLOSE_POWER), (interrupt) -> setPower(0), () -> isClosed(), this);
  }

  public void toggle() {
    
  }
}
