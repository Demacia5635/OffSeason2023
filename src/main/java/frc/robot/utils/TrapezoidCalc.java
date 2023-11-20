// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;


import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TrapezoidCalc {
    double lastVel = 0;
    double lastAcc = 0;
    double lastTime = 0;
    
    public TrapezoidCalc(){
    }

    public double trapezoid(double currentVelocity, double maxVel, double endVel, double acc, double dis){
        double time = Timer.getFPGATimestamp();
        System.out.print("Trapezoid: v =" + currentVelocity + " maxv =" + maxVel + " acc =" + acc + "dis =" + dis);
        if(time - lastTime < 0.04) {
            if(lastAcc > 0 && currentVelocity < lastVel) {
                currentVelocity = lastVel;
            }
            if(lastAcc < 0 && currentVelocity > lastVel) {
                currentVelocity = lastVel;
            }
        }
        double timeToAccelerate = (currentVelocity-endVel)/acc;
        System.out.print(" cv =" + currentVelocity);
        if(dis > 0) {
            double accelDistance = currentVelocity*timeToAccelerate + acc*Math.pow(timeToAccelerate,2)/2;
            System.out.print(" accd =" + accelDistance);
            if(dis > accelDistance) {
                lastVel = Math.min(currentVelocity + 0.02*acc, maxVel);
            } else {
                lastVel = currentVelocity - acc*0.02;
            }
        } else {
            double accelDistance = currentVelocity*timeToAccelerate - acc*Math.pow(timeToAccelerate,2)/2;
            System.out.print(" accd =" + accelDistance);
            if(dis < accelDistance) {
                lastVel =  Math.max(currentVelocity - 0.02*acc, -maxVel);
            } else {
                lastVel = currentVelocity + acc*0.02;
            }
        }
        lastTime = time;
        lastAcc = lastVel - currentVelocity;
        System.out.println(" lastv =" + lastVel + " lastAcc =" + lastAcc);
        return lastVel;
    }
}