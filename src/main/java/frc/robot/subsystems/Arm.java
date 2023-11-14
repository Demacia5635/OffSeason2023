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

import static frc.robot.Constants.ArmConstants.*;




public class Arm extends SubsystemBase {
  
  public TalonFX motor = new TalonFX(motorID);
  public double baseAngle = 0;
  double lastVel = 0;

  public DigitalInput input = new DigitalInput(DigitalInputID);

  /** Creates a new Parraller. */
  public Arm() {
    
    SmartDashboard.putData(this);
  }

  public void setPow(double AP){
    motor.set(ControlMode.PercentOutput, AP);
  }
  
  public void stop(){
    setPow(0);
  }

  public double FF(double wantedAnglerVel){
    double rad = Math.toRadians(getAngle());
    int state;
    
    if (wantedAnglerVel > 0){
      if (getAngle() <= 43){
        state = 1;
      } else {
        state = 2;
      }
    } else {
      if (getAngle() <= 43){
        state = 3;
      } else {
        state = 4;
      }
    }

    return (
      KS[state] + 
      wantedAnglerVel * KV[state] + 
      (wantedAnglerVel-getCurrentAnglerVel()) * KA[state] + 
      Kalpha[state] * rad + 
      Ksin[state] * Math.sin(rad) + 
      Kcos[state] * Math.cos(rad) + 
      Kcossin[state] * Math.cos(rad) * Math.sin(rad)
    );
  }

  public void setVel(double wantedAnglerVel){
    motor.set(ControlMode.Velocity, wantedAnglerVel, DemandType.ArbitraryFeedForward, FF(wantedAnglerVel));
  }
  
  // if false will stop the command; false when colide with the arm;
  public boolean getInput(){ return !input.get(); }
  public double getCurrentAnglerVel(){ return motor.getSelectedSensorVelocity() * 10 / pulsePerAngle; }
  public double getAngle(){ return (motor.getSelectedSensorPosition() / pulsePerAngle) - baseAngle; }
  public double getPow(){ return motor.getMotorOutputPercent(); }
  public double getValt(){ return motor.getMotorOutputVoltage(); }
  public double getVelAcc(){ 
    double returnn = (getCurrentAnglerVel()-lastVel);
    lastVel = getCurrentAnglerVel();
    return returnn;
  }

  

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

      builder.addDoubleProperty("Current Angle Velocity", this::getCurrentAnglerVel, null);
      builder.addDoubleProperty("Angle", this::getAngle, null);
      builder.addBooleanProperty("Input", this::getInput, null);
      builder.addDoubleProperty("Pow", this::getPow, null);
      builder.addDoubleProperty("Valt", this::getValt, null);
      builder.addDoubleProperty("Velocity accelaration", this::getVelAcc, null);

      // SmartDashboard.putNumber("KS", KS);
      // SmartDashboard.putNumber("KV", KV);
      // SmartDashboard.putNumber("KA", KA);
      // SmartDashboard.putNumber("Ksin", Ksin);
      // SmartDashboard.putNumber("Kcos", Kcos);
      // SmartDashboard.putNumber("Kcossin", Kcossin);
      // SmartDashboard.putNumber("Kalpha", Kalpha);

      InstantCommand cmdBrake = new InstantCommand(()-> brake(), this);
      InstantCommand cmdCoast = new InstantCommand(()-> coast(), this);
      SmartDashboard.putData("Brake", cmdBrake.ignoringDisable(true));
      SmartDashboard.putData("Coast", cmdCoast.ignoringDisable(true));
  }

}
