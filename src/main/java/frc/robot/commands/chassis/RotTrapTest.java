// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import java.util.concurrent.ThreadPoolExecutor.DiscardOldestPolicy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PathFollow.Util.TrapezShay;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.TrapezoidNoam;

public class RotTrapTest extends CommandBase {
  /** Creates a new RotTrapTest. */
  double rotationVelocity;
  TrapezoidNoam rotationTrapezoid;
  Chassis chassis;
  double distanceLeft;
  double prevVecloity;
  PIDController pidController;

  public RotTrapTest(Chassis chassis) {
    this.chassis= chassis;
    rotationTrapezoid = new TrapezoidNoam(360, 720);
    addRequirements(chassis);
    SmartDashboard.putData(this);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prevVecloity = 0;  
    rotationVelocity = 360;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     distanceLeft = Rotation2d.fromDegrees(179).minus(chassis.getAngle()).getDegrees();
    //rotationVelocity = -rotationTrapezoid.calculate(distanceLeft, Math.toDegrees(chassis.getChassisSpeeds().omegaRadiansPerSecond), 0);
    

    chassis.setVelocities(new ChassisSpeeds(0, 0, Math.toRadians(rotationVelocity)));
    
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("ROTATION VELOCITY",()-> {return rotationVelocity;}, null);
      builder.addDoubleProperty("CHASSIS VELOCITY",()-> {return Math.toDegrees(chassis.getChassisSpeeds().omegaRadiansPerSecond);}, null);
      builder.addDoubleProperty("CURRENT ANGLE",()->  {return chassis.getAngle().getDegrees();}, null);
      builder.addDoubleProperty("DISTANCE LEFT", ()-> {return distanceLeft;}, null);
  }
  // Called once the command ends or is interrupted.
  @Override 
  public void end(boolean interrupted) {
    chassis.stop();
  }

  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
