package frc.robot.subsystems.chassis;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.chassis.utils.SwerveModule;

import static frc.robot.Constants.ChassisConstants.*;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
      module.setAnglePID(ANGLE_VELOCITY_KP, ANGLE_VELOCITY_KI, ANGLE_VELOCITY_KP);
    });

    gyro = new PigeonIMU(GYRO_ID);

    poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getAngle(), getModulePositions(), new Pose2d());
    field = new Field2d();
    SmartDashboard.putData(field);
    SmartDashboard.putData(this);
    SmartDashboard.putData("left front module", modules[0]);
    SmartDashboard.putData("right front module", modules[1]);
    SmartDashboard.putData("left back module", modules[2]);
    SmartDashboard.putData("right back module", modules[3]);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("chassis velocity", () -> getVelocity().getNorm(), null);
  }

  public void stop() {
    Arrays.stream(modules).forEach(SwerveModule::stop);
  }

  public void setDrivePower(double power) {
    Arrays.stream(modules).forEach((module) -> module.setPower(power, 0));
  }

  public void setDriveVelocity(double velocity) {
    Arrays.stream(modules).forEach((module) -> module.setVelocity(velocity));
  }

  public void setVelocities(ChassisSpeeds speeds) {
    ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
    SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(relativeSpeeds);
    setModuleStates(states);
  }

  public Translation2d getVelocity() {
    ChassisSpeeds speeds = KINEMATICS.toChassisSpeeds(getModuleStates());
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public void resetWheels() {
    Arrays.stream(modules).forEach((module) -> module.setAngle(new Rotation2d(0)));
  }

  public void setWheelAngles(double x) {
    for (SwerveModule module : modules)
      module.setAngle(Rotation2d.fromDegrees(x));
  }

  public void setNeutralMode(NeutralMode mode) {
    Arrays.stream(modules).forEach((module) -> module.setNeutralMode(mode));
  }

  @Override
  public void periodic() {
      poseEstimator.update(getAngle(), getModulePositions());
      field.setRobotPose(poseEstimator.getEstimatedPosition());
      
      Arrays.stream(modules).forEach((module) -> module.update());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(gyro.getFusedHeading());
  }

  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(modules).map(SwerveModule::getModulePosition).toArray(SwerveModulePosition[]::new);
  }

  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < 4; i++) modules[i].setState(states[i]);
  }
}
