package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GripperConstants.*;


public class Gripper extends SubsystemBase {
  public TalonSRX motor;

  public Gripper() {
    motor = new TalonSRX(MOTOR_ID);
  }

  private boolean isClosed() {
    return motor.isFwdLimitSwitchClosed() == 1;
  }

  private boolean isOpened() {
    return motor.isRevLimitSwitchClosed() == 1;
  }

  public void setPower(double pow) {
    motor.set(ControlMode.PercentOutput, pow);
  }

  public void stop() {
    setPower(0);
  }

  public void open() {
    new FunctionalCommand(null, () -> setPower(OPEN_POWER), (interrupt) -> stop(), () -> isOpened(), this);
  }

  public void close() {
    new FunctionalCommand(null, () -> setPower(CLOSE_POWER), (interrupt) -> stop(), () -> isClosed(), this);
  }

  public GripperState getState() {
    if (isClosed() && !isOpened()){
      return GripperState.CLOSE;
    } else if(isOpened() && !isClosed()){
      return GripperState.OPEN;
    } else {
      return GripperState.BETWEEN;
    }
  }

  enum GripperState{
    CLOSE,
    OPEN,
    BETWEEN
  }
}
