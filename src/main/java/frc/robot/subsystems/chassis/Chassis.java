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
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    SmartDashboard.putData(this);
    SmartDashboard.putData("m1 offset", modules[0]);
    SmartDashboard.putData("m2 offset", modules[1]);
    SmartDashboard.putData("m3 offset", modules[2]);
    SmartDashboard.putData("m4 offset", modules[3]);

    modules[0].setInverted(false);
    modules[1].setInverted(true);
    modules[2].setInverted(false);
    modules[3].setInverted(true);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("vel", () -> modules[2].getVelocity(), null);
  }

  public SwerveDrivePoseEstimator getPose(){
    return poseEstimator;
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

  public ChassisSpeeds getVelocity() {
    return KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void resetWheels() {
    Arrays.stream(modules).forEach((module) -> module.setAngle(new Rotation2d(0)));
  }

  public void setWheelAngles(double x) {
    Arrays.stream(modules).forEach((module) -> module.setAngle(Rotation2d.fromDegrees(x)));
  }

  @Override
  public void periodic() {
      poseEstimator.update(getAngle(), getModulePositions());
      field.setRobotPose(poseEstimator.getEstimatedPosition());
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
