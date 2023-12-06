package frc.robot.subsystems.chassis;

import frc.robot.subsystems.chassis.vision.utils.SwerveDrivePoseEstimator;
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
import frc.robot.subsystems.chassis.vision.Vision;
import frc.robot.subsystems.chassis.vision.utils.SwerveDrivePoseEstimator;

import static frc.robot.Constants.ChassisConstants.*;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Chassis extends SubsystemBase {
  private final SwerveModule[] modules;
  private final PigeonIMU gyro;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field;
  
  Vision vision;

  public Chassis() {

    modules = new SwerveModule[] {
      new SwerveModule(MODULE_FRONT_LEFT),
      new SwerveModule(MODULE_FRONT_RIGHT),
      new SwerveModule(MODULE_BACK_LEFT),
      new SwerveModule(MODULE_BACK_RIGHT),
    };

    gyro = new PigeonIMU(GYRO_ID);

    poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getAngle(), getModulePositions(), new Pose2d());
    poseEstimator.resetPosition(new Rotation2d(0), getModulePositions(), new Pose2d());
    vision = new Vision(this, poseEstimator); 
    field = new Field2d();
    SmartDashboard.putData(field);
    SmartDashboard.putData(this);
    SmartDashboard.putData("left front module", modules[0]);
    SmartDashboard.putData("right front module", modules[1]);
    SmartDashboard.putData("left back module", modules[2]);
    SmartDashboard.putData("right back module", modules[3]);

    SmartDashboard.putData("set coast", new InstantCommand(() -> setNeutralMode(NeutralMode.Coast)).ignoringDisable(true));
    SmartDashboard.putData("set brake", new InstantCommand(() -> setNeutralMode(NeutralMode.Brake)).ignoringDisable(true));

    SmartDashboard.putData("reset wheels", new InstantCommand(() -> {
      for (SwerveModule module : modules) {
        module.setAngle(new Rotation2d());
      }
    }));

      SmartDashboard.putData("reset pose", new InstantCommand(()-> resetPose()));
  }
  

  public double getPoseX(){
    return poseEstimator.getEstimatedPosition().getX();
  }

  public double getPoseY(){
    return poseEstimator.getEstimatedPosition().getY();
  }
  
  public SwerveDrivePoseEstimator getPose(){
    return poseEstimator;
  }

  public void resetPose(){
    poseEstimator.resetPosition(new Rotation2d(0), getModulePositions(), new Pose2d());
    gyro.setFusedHeading(0);
  }

  /**
   * Stops the entire chassis
   */
  public void stop() {
    Arrays.stream(modules).forEach(SwerveModule::stop);
  }


  public boolean timeAfterLastUpdate(){
    return vision.timeAfterLastUpdate();
  }

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
    ChassisSpeeds speeds = KINEMATICS.toChassisSpeeds(getModuleStates());
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
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

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("chassis velocity", () -> getVelocity().getNorm(), null);
      builder.addDoubleProperty("pose X",() -> getPoseX(), null);
      builder.addDoubleProperty("pose Y",() -> getPoseY(), null);
      builder.addDoubleProperty("Angle", () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees(), null);
      builder.addDoubleProperty("Angle speed", () -> Math.toDegrees(getVelocity().omegaRadiansPerSecond), null);
  }

  @Override
  public void periodic() {
      poseEstimator.update(getAngle(), getModulePositions());
      field.setRobotPose(poseEstimator.getEstimatedPosition());
  }
}
