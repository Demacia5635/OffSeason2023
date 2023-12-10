package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;

public class FollowPathCommand extends CommandBase {
   private final Chassis chassis;
   private final Trajectory trajectory;

   public FollowPathCommand(Chassis chassis, Path path) {
       this.chassis = chassis;
       try {
           this.trajectory = TrajectoryUtil.fromPathweaverJson(path);
       } catch (IOException e) {
           throw new RuntimeException("Unable to load path", e);
       }

       addRequirements(chassis);
   }

   @Override
   public void initialize() {
       
   }

   @Override
   public void execute() {
       Trajectory.State state = trajectory.sample(chassis.getTime());
       chassis.setDriveVelocity(state.velocityMetersPerSecond);
       chassis.setWheelAngles(state.poseMeters.getRotation().getDegrees());
   }

   @Override
   public void end(boolean interrupted) {
       chassis.stop();
   }

   @Override
   public boolean isFinished() {
       return chassis.getTime() >= trajectory.getTotalTimeSeconds();
   }
}