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
  public DigitalInput input = new DigitalInput(DigitalInputID);
  private double requiredVelocity = 0;
  private double calculatedPower = 0;

  /** Creates a new Parraller. */
  public Arm() {
    
    SmartDashboard.putData(this);
  }

  public void setPow(double wantedPow){
    motor.set(ControlMode.PercentOutput, wantedPow);
  }
  
  public void stop(){
    setPow(0);
  }
  
  public int state;
  public double FF(double wantedAnglerVel){
    double angle = getAngle();
    double rad = Math.toRadians(angle);
    
    if (wantedAnglerVel > 0){
      if (angle <= 43){
        state = 0;
      } else {
        state = 1;
      }
    } else /*if (wantedAngelerVel < 0)*/{
      if (getAngle() <= 43){
        state = 2;
      } else {
        state = 3;
      }
    }

    return (
      KS[state] + 
      wantedAnglerVel * KV[state] + 
      (wantedAnglerVel-getCurrentAnglerVel()) * KA[state] + 
      Kalpha[state] * angle + 
      Ksin[state] * Math.sin(rad) +   
      Kcos[state] * Math.cos(rad) + 
      Kcossin[state] * Math.cos(rad) * Math.sin(rad)
    );
  }

  public void setVel(double wantedAnglerVel){
    requiredVelocity = wantedAnglerVel;
    calculatedPower = FF(wantedAnglerVel);
    System.out.println("arm angle = " + getAngle());
    System.out.println("arm current vel = " + getCurrentAnglerVel());
    System.out.println("arm velocity = " + wantedAnglerVel);
    System.out.println("arm FF = " + calculatedPower);
    // motor.set(ControlMode.PercentOutput, calculatedPower);
    motor.set(ControlMode.Velocity, wantedAnglerVel*pulsePerAngle/10, DemandType.ArbitraryFeedForward, calculatedPower);
  }
  
  // get flase from the limit switch when close reverse it in this command
  public boolean getInput(){ return !input.get(); }
  public double getCurrentAnglerVel(){ return motor.getSelectedSensorVelocity() * 10 / pulsePerAngle; }

  public double baseAngle = 0;
  public double getAngle(){ return (motor.getSelectedSensorPosition() / pulsePerAngle) - baseAngle; }
  public double getPow(){ return motor.getMotorOutputPercent(); }
  public double getVolt(){ return motor.getMotorOutputVoltage(); }
  
  double lastVel = 0;
  public double getVelAcc(){ 
    double returnn = (lastVel-getCurrentAnglerVel());
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
      super.initSendable(builder);

      builder.addDoubleProperty("Current Angle Velocity", this::getCurrentAnglerVel, null);
      builder.addDoubleProperty("Angle", this::getAngle, null);
      builder.addBooleanProperty("Input", this::getInput, null);
      builder.addDoubleProperty("Pow", this::getPow, null);
      builder.addDoubleProperty("Volt", this::getVolt, null);
      builder.addDoubleProperty("Accelaration", this::getVelAcc, null);
      builder.addDoubleProperty("state", ()->state, null);
      builder.addDoubleProperty("wanted velocity", ()->requiredVelocity, null);
      builder.addDoubleProperty("feed forward", ()->calculatedPower, null);

      InstantCommand cmdBrake = new InstantCommand(()-> brake(), this);
      InstantCommand cmdCoast = new InstantCommand(()-> coast(), this);
      SmartDashboard.putData("Brake", cmdBrake.ignoringDisable(true));
      SmartDashboard.putData("Coast", cmdCoast.ignoringDisable(true));
  }

}
