// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ParallelogramConstants.*;




public class Parallelogram extends SubsystemBase {
  
  public TalonFX motor = new TalonFX(motorID);
  public double baseAngle = 0;

  public DigitalInput input = new DigitalInput(DigitalInputID);

  /** Creates a new Parraller. */
  public Parallelogram() {
    motor.config_kP(0,KP);
    motor.config_kP(0,KI);
    motor.config_kP(0,KD);
    
    SmartDashboard.putData(this);
  }

  public void setPow(double pow){
    motor.set(ControlMode.PercentOutput, pow);
  }
  
  public void stop(){
    setPow(0);
  }

  public double FF(double AV, double angle, double CAV){
    double rad = Math.toRadians(angle);
    double P = KS + AV * KV + (AV-CAV) * KA + Kalpha * rad + Ksin * Math.sin(rad) + Kcos*Math.cos(rad) + Kcossin * Math.cos(rad) * Math.sin(rad);
    return P;
  }

  public void setVel(double AV){
    motor.set(ControlMode.Velocity, AV, DemandType.ArbitraryFeedForward, FF(getCAV(),getAngle(),AV));
  }

  public boolean getInput(){ return input.get(); }// if true will stop the command
  public boolean isRetracted(){return false;}

  public double getCAV(){ return motor.getSelectedSensorVelocity() * 10 / pulsePerAngle; }
  public double getAngle(){ return (motor.getSelectedSensorPosition() / pulsePerAngle) - baseAngle; }
  public double getValtPercentage(){ return motor.getMotorOutputPercent(); }
  public double getValtValtage(){ return motor.getMotorOutputVoltage(); }
  

  public void brake(){
    motor.setNeutralMode(NeutralMode.Brake);
  }
  public void coast(){
    motor.setNeutralMode(NeutralMode.Coast);
  }


  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);

      builder.addBooleanProperty("isRetracted", this::isRetracted, null);
      builder.addDoubleProperty("Current Angle Velocity", this::getCAV, null);
      builder.addDoubleProperty("Angle", this::getAngle, null);
      builder.addBooleanProperty("Input", this::getInput, null);
      builder.addDoubleProperty("Pow", this::getValtPercentage, null);
      builder.addDoubleProperty("Valt", this::getValtValtage, null);
      

      SmartDashboard.putNumber("KP", KP);
      SmartDashboard.putNumber("KI", KI);
      SmartDashboard.putNumber("KD", KD);

      SmartDashboard.putNumber("KS", KS);
      SmartDashboard.putNumber("KV", KV);
      SmartDashboard.putNumber("KA", KA);
      SmartDashboard.putNumber("Ksin", Ksin);
      SmartDashboard.putNumber("Kcos", Kcos);
      SmartDashboard.putNumber("Kcossin", Kcossin);
      SmartDashboard.putNumber("Kalpha", Kalpha);

      InstantCommand cmdBrake = new InstantCommand(()-> brake(), this);
      InstantCommand cmdCoast = new InstantCommand(()-> coast(), this);
      SmartDashboard.putData("Brake", cmdBrake.ignoringDisable(true));
      SmartDashboard.putData("Coast", cmdCoast.ignoringDisable(true));
  }

}