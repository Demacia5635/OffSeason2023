package frc.robot.subsystems.chassis;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.chassis.utils.SwerveModule;

import static frc.robot.Constants.ChassisConstants.*;

import java.util.Arrays;

import com.ctre.phoenix.sensors.PigeonIMU;

public class Chassis extends SubsystemBase {
  private final SwerveModule[] modules;
  private final PigeonIMU gyro;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field;

  public Chassis() {
    modules = new SwerveModule[] {
      new SwerveModule(MODULE_FRONT_LEFT),
      new SwerveModule(MODULE_FRONT_RIGHT),
      new SwerveModule(MODULE_BACK_LEFT),
      new SwerveModule(MODULE_BACK_RIGHT),
    };
    Arrays.stream(modules).forEach((module) -> {
      module.setMovePID(MOVE_KP, MOVE_KI, MOVE_KD);
      module.setAnglePID(ANGLE_KP, ANGLE_KI, ANGLE_KD);
    });

    gyro = new PigeonIMU(GYRO_ID);

    poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getAngle(), getModulePositions(), new Pose2d());
    field = new Field2d();
    SmartDashboard.putData(field);
  }

  public void stop() {
    Arrays.stream(modules).forEach(SwerveModule::stop);
  }

  public void setDrivePower(double power) {
    Arrays.stream(modules).forEach((module) -> module.setPower(power));
  }

  public void setDriveVelocity(double velocity) {
    Arrays.stream(modules).forEach((module) -> module.setVelocity(velocity));
  }

  public void setVelocities(ChassisSpeeds speeds) {
    ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
    SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(relativeSpeeds);
    for (int i = 0; i < 4; i++) modules[i].setState(states[i]);
  }

  @Override
  public void periodic() {
      poseEstimator.update(getAngle(), getModulePositions());
      field.setRobotPose(poseEstimator.getEstimatedPosition());
  }

  public Translation2d getVelocity() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
        states[i] = new SwerveModuleState(modules[i].getVelocity(), modules[i].getAngle());
    }
    ChassisSpeeds speeds = KINEMATICS.toChassisSpeeds(states);
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
}


  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(gyro.getFusedHeading());
  }

  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(modules).map(SwerveModule::getModulePosition).toArray(SwerveModulePosition[]::new);
  }
}
