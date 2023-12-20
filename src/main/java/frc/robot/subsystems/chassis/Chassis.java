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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.chassis.utils.ResetWheelCommand;
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

    gyro = new PigeonIMU(GYRO_ID);

    poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getAngle(), getModulePositions(), new Pose2d());
    field = new Field2d();
    SmartDashboard.putData(field);
    SmartDashboard.putData(this);
    SmartDashboard.putData("left front module", modules[0]);
    SmartDashboard.putData("right front module", modules[1]);
    SmartDashboard.putData("left back module", modules[2]);
    SmartDashboard.putData("right back module", modules[3]);

    modules[0].setInverted(false);
    modules[1].setInverted(false);
    modules[2].setInverted(false);
    modules[3].setInverted(false);

    SmartDashboard.putData("set coast", new InstantCommand(() -> setNeutralMode(NeutralMode.Coast)).ignoringDisable(true));
    SmartDashboard.putData("set brake", new InstantCommand(() -> setNeutralMode(NeutralMode.Brake)).ignoringDisable(true));

    SmartDashboard.putData("reset wheels", new RunCommand(() -> {
      for (SwerveModule module : modules) {
        module.setAngle(new Rotation2d());
      }
    }).ignoringDisable(true));
  }

  public void resetWheels() {
    Command cmd = new ResetWheelCommand(modules[0]);
    cmd = cmd.alongWith(new ResetWheelCommand(modules[1]));
    cmd = cmd.alongWith(new ResetWheelCommand(modules[2]));
    cmd = cmd.alongWith(new ResetWheelCommand(modules[3]));

    cmd.schedule();
  }

  /**
   * Stops the entire chassis
   */
  public void stop() {
    Arrays.stream(modules).forEach(SwerveModule::stop);
  }

  /**
   * Sets the velocity of the chassis
   * @param speeds In m/s and rad/s
   */
  public void setVelocities(ChassisSpeeds speeds) {
    ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
    SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(relativeSpeeds);
    setModuleStates(states);
  }

  /**
   * Returns the velocity vector
   * @return Velocity in m/s
   */
  public Translation2d getVelocity() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  /**
   * Sets the neutral mode of every motor in the chassis
   * @param mode
   */
  public void setNeutralMode(NeutralMode mode) {
    Arrays.stream(modules).forEach((module) -> module.setNeutralMode(mode));
  }

  /**
   * Returns the angle of the gyro
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(gyro.getFusedHeading());
  }

  /**
   * Returns the position of every module
   * @return Position relative to the field
   */
  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(modules).map(SwerveModule::getModulePosition).toArray(SwerveModulePosition[]::new);
  }

  /**
   * Returns the state of every module
   * @return Velocity in m/s, angle in Rotation2d
   */
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
  }

  /**
   * Sets the state of every module
   * @param states Velocity in m/s, angle in Rotation2d
   */
  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < 4; i++) modules[i].setState(states[i]);
  }

  public void setModulesAngles(double angle) {
    Rotation2d rot = Rotation2d.fromDegrees(angle);
    SwerveModuleState[] currentStates = getModuleStates();
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = new SwerveModuleState(currentStates[0].speedMetersPerSecond, rot);
    states[1] = new SwerveModuleState(currentStates[1].speedMetersPerSecond, rot);
    states[2] = new SwerveModuleState(currentStates[2].speedMetersPerSecond, rot);
    states[3] = new SwerveModuleState(currentStates[3].speedMetersPerSecond, rot);
    for (int i = 0; i < 4; i++) modules[i].setState(states[i]);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void updateField() {
    field.setRobotPose(getPose());
  }

  private void resetPose(){
    poseEstimator.resetPosition(new Rotation2d(0), getModulePositions(), new Pose2d());
    gyro.setFusedHeading(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("chassis velocity", () -> getVelocity().getNorm(), null);
      builder.addDoubleProperty("chassis ang velocity", () -> Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond), null);
      SmartDashboard.putData("reset pose", new InstantCommand(()-> resetPose()));
  }


  @Override
  public void periodic() {
      poseEstimator.update(getAngle(), getModulePositions());
      updateField();
  }
}
