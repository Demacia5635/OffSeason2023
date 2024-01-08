// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import java.util.concurrent.ThreadPoolExecutor.DiscardOldestPolicy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PathFollow.Util.Trapez;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.Trapezoid;

public class RotTrapTest extends CommandBase {
  /** Creates a new RotTrapTest. */
  double rotationVelocity;
  Trapezoid rotationTrapezoid;
  Chassis chassis;
  double distanceLeft;
  public RotTrapTest(Chassis chassis) {
    this.chassis= chassis;
    rotationTrapezoid = new Trapezoid(100, 200);
    addRequirements(chassis);
    SmartDashboard.putData(this);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     distanceLeft = Rotation2d.fromDegrees(50).minus(chassis.getAngle()).getDegrees();
    rotationVelocity = -rotationTrapezoid.calculate(distanceLeft, chassis.getChassisSpeeds().omegaRadiansPerSecond, 0);
    
    chassis.setVelocities(new ChassisSpeeds(0, 0, rotationVelocity));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("ROTATION VELOCITY",()-> {return rotationVelocity;}, null);
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
    return distanceLeft <= 1 && distanceLeft >= -1;
  }
}
