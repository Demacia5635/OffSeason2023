package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static final double CYCLE_DT = 0.02;

  public static final double segmentID = 1;
  static Pose2d aprilTag1 = new Pose2d(15.513558, 1.071626, new Rotation2d(180));
  static Pose2d aprilTag2 = new Pose2d(15.513558, 2.748026, new Rotation2d(180));
  static Pose2d aprilTag3 = new Pose2d(15.513558, 4.424426, new Rotation2d(180));
  static Pose2d aprilTag4 = new Pose2d(16.178784, 6.749796, new Rotation2d(180));
  static Pose2d aprilTag5 = new Pose2d(0.36195, 6.749796, new Rotation2d());
  static Pose2d aprilTag6 = new Pose2d(1.02743,  4.424426, new Rotation2d());
  static Pose2d aprilTag7 = new Pose2d(1.02743, 2.748026, new Rotation2d());
  static Pose2d aprilTag8 = new Pose2d(1.02743, 1.071626, new Rotation2d());
  public final static Pose2d[] aprilTagsPositions = {aprilTag1, aprilTag2, aprilTag3, aprilTag4, aprilTag5, aprilTag6, aprilTag7, aprilTag8};


  public static class ChassisConstants {
    public final static SwerveModuleConstants MODULE_FRONT_LEFT = new SwerveModuleConstants(
      7, 8, 11, 0, 0, 0,
      new Translation2d(0.26515, 0.2215),
      27.59765625
    );
    public final static SwerveModuleConstants MODULE_FRONT_RIGHT = new SwerveModuleConstants(
      5, 6, 13, 0, 0, 0,
      new Translation2d(0.26515, -0.2215),
      199.16015625
    );
    public final static SwerveModuleConstants MODULE_BACK_LEFT = new SwerveModuleConstants(
      1, 2, 10, 0, 0, 0,
      new Translation2d(-0.25451, 0.2065),
      228.69140625
    );
    public final static SwerveModuleConstants MODULE_BACK_RIGHT = new SwerveModuleConstants(
      3, 4, 12, 0, 0, 0,
      new Translation2d(-0.26515, -0.2215),
      212.607421875
    );
    public static final int GYRO_ID = 14;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      MODULE_FRONT_LEFT.moduleTranslationOffset,
      MODULE_FRONT_RIGHT.moduleTranslationOffset,
      MODULE_BACK_LEFT.moduleTranslationOffset,
      MODULE_BACK_RIGHT.moduleTranslationOffset
    );

    public static final double VELOCITY = 4;
    public static final double ACCELERATION = 8;
    public static final double ANGULAR_VELOCITY = 360;
    public static final double ANGULAR_ACCELERATION = 720;

    public static final double MOVE_KP = 0.27;
    public static final double MOVE_KI = 0.0;
    public static final double MOVE_KD = 0.12;
    public static final double ANGLE_KP = 0.1;
    public static final double ANGLE_KI = 0;
    public static final double ANGLE_KD = 0;

    public static final double PULSES_PER_METER = 52226.56641604010025062656641604;
    public static final double PULSES_PER_DEGREE = 72.817777777777777777777777777779;

    public static class SwerveModuleConstants {
      public final int moveMotorId;
      public final int angleMotorId;
      public final int absoluteEncoderId;
      public final int kS;
      public final int kV;
      public final int kA;
      public final Translation2d moduleTranslationOffset;
      public final double steerOffset;

      public SwerveModuleConstants(int moveMotorId, int angleMotorId, int absoluteEncoderId, int kS, int kV, int kA, Translation2d moduleTranslationOffset, double steerOffset) {
        this.moveMotorId = moveMotorId;
        this.angleMotorId = angleMotorId;
        this.absoluteEncoderId = absoluteEncoderId;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.moduleTranslationOffset = moduleTranslationOffset;
        this.steerOffset = steerOffset;
      }
    }
  }
}
