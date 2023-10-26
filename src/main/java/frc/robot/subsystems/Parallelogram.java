// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ParallelogramConstants.*;


public class Parallelogram extends SubsystemBase {
  
  public TalonFX motor = new TalonFX(motorID);
  public DigitalInput input = new DigitalInput(DigitalInputID);

  /** Creates a new Parraller. */
  public Parallelogram() {
    SmartDashboard.putData(this);
  }

  public void setPow(double pow){
    motor.set(ControlMode.PercentOutput, pow);
  }

  public void setVel(double vel){
    motor.set(ControlMode.Velocity, vel);
  }

  public void stop(){
    setPow(0);
  }

  public boolean isRetracted(){return false;}

  public double getVel(){ return motor.getSelectedSensorVelocity() * 10 / pulsePerAngle; }
  public double getAngle(){ return motor.getSelectedSensorPosition() / pulsePerAngle; }


  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);

      builder.addBooleanProperty("isRetracted", this::isRetracted, null);
      builder.addDoubleProperty("Velocity", this::getVel, null);
      builder.addDoubleProperty("Angle", this::getAngle, null);

      SmartDashboard.putNumber("KP", KP);
      SmartDashboard.putNumber("KI", KI);
      SmartDashboard.putNumber("KD", KD);
  }

  double sumError = 0;
  double lastError = 0;
  double error;
  double p,i,d;
  double pv;

  public double getPID(double sp){
    pv = getVel();
    error = sp - pv;
    sumError += error;
    p = KP * error;
    i = KI * sumError;
    d = KD * (lastError - error);
    lastError = error;
    return p+i+d;
  }
}
