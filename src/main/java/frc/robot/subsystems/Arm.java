// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
  
    public TalonFX motor = new TalonFX(motorID);
    public DigitalInput limitSwitch = new DigitalInput(digitalInputID);
    public double baseAngle = 0;

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
        } else {
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
        motor.set(ControlMode.Velocity, wantedAnglerVel*pulsePerAngle/10, DemandType.ArbitraryFeedForward, FF(wantedAnglerVel));
    }
    
    // get flase from the limit switch when close reverse it in this command
    public boolean getLimitSwitch(){ return !limitSwitch.get(); }
    public double getCurrentAnglerVel(){ return motor.getSelectedSensorVelocity() * 10 / pulsePerAngle; }
    public double getAngle(){ return (motor.getSelectedSensorPosition() / pulsePerAngle) - baseAngle; }

    // public double getPow(){ return motor.getMotorOutputPercent(); }
    // public double getVolt(){ return motor.getMotorOutputVoltage(); }
    
    // double lastVel = 0;
    // public double getVelAcc(){ 
    //     double returnn = (lastVel-getCurrentAnglerVel());
    //     lastVel = getCurrentAnglerVel();
    //     return returnn;
    // }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Arm Current Angle Velocity", this::getCurrentAnglerVel, null);
        builder.addDoubleProperty("Arm Angle", this::getAngle, null);
        builder.addBooleanProperty("Arm Limit switch", this::getLimitSwitch, null);
        builder.addIntegerProperty("Arm state", ()->state, null);
  }
}
