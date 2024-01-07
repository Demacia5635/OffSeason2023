package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static final double CYCLE_DT = 0.02;

  public static class ChassisConstants {
    public final static SwerveModuleConstants MODULE_FRONT_LEFT = new SwerveModuleConstants(
      7, 8, 11,
      new Translation2d(0.26515, 0.2215),
      216.650390625
    );
    public final static SwerveModuleConstants MODULE_FRONT_RIGHT = new SwerveModuleConstants(
      5, 6, 13,
      new Translation2d(0.26515, -0.2215),
      197.2265625
    );
    public final static SwerveModuleConstants MODULE_BACK_LEFT = new SwerveModuleConstants(
      1, 2, 10,
      new Translation2d(-0.25451, 0.2065),
      50.2734375
    );
        public final static SwerveModuleConstants MODULE_BACK_RIGHT = new SwerveModuleConstants(
      3, 4, 12,
      new Translation2d(-0.26515, -0.2215),
      215.859375
    );
    public static final int GYRO_ID = 14;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      MODULE_FRONT_LEFT.moduleTranslationOffset,
      MODULE_FRONT_RIGHT.moduleTranslationOffset,
      MODULE_BACK_LEFT.moduleTranslationOffset,
      MODULE_BACK_RIGHT.moduleTranslationOffset
    );

    public static final double MAX_DRIVE_VELOCITY = 1;
    public static final double DRIVE_ACCELERATION = 4;
    public static final double MAX_ANGULAR_VELOCITY = 360;
    public static final double ANGULAR_ACCELERATION = 720;

    public static final double PULSES_PER_METER = 52226.56641604010025062656641604;
    public static final double PULSES_PER_DEGREE = 72.817777777777777777777777777779;

    public static class SwerveModuleConstants {
      public static final double MOVE_KP = 0.002;
      public static final double MOVE_KI = 0.0008;
      public static final double MOVE_KD = 0.016;
      public static final double ANGLE_POSITION_KP = 0.35;
      public static final double ANGLE_POSITION_KI = 0;
      public static final double ANGLE_POSITION_KD = 0.029;
      public static final double ANGLE_VELOCITY_KP = 0.07;
      public static final double ANGLE_VELOCITY_KI = 0.004;
      public static final double ANGLE_VELOCITY_KD = 0.05;

      public static final double MOVE_KS = 0.0362;
      public static final double MOVE_KV = 0.0862;
      public static final double ANGLE_KS = 0.05;
      public static final double ANGLE_KV = 0.0962;

      public final int moveMotorId;
      public final int angleMotorId;
      public final int absoluteEncoderId;
      public final Translation2d moduleTranslationOffset;
      public final double steerOffset;

      public SwerveModuleConstants(int moveMotorId, int angleMotorId, int absoluteEncoderId, Translation2d moduleTranslationOffset, double steerOffset) {
        this.moveMotorId = moveMotorId;
        this.angleMotorId = angleMotorId;
        this.absoluteEncoderId = absoluteEncoderId;
        this.moduleTranslationOffset = moduleTranslationOffset;
        this.steerOffset = steerOffset;
      }
    }
  }

  public static final class VisionConstants {

    public static final String Limelight2Name = "Limelight2";
    public static final String Limelight3Name = "Limelight3";

    public static final Pose2d robotCenterToLimelight2 = new Pose2d(new Translation2d(0.14, -0.22),Rotation2d.fromDegrees(-28));
    public static final Transform3d robotCenterToLimelight2Transform = new Transform3d(new Pose3d(), new Pose3d(robotCenterToLimelight2));

    public static final Pose2d robotCenterToLimelight3 = new Pose2d(new Translation2d(-0.14, -0.22),Rotation2d.fromDegrees(28));
    public static final Transform3d robotCenterToLimelight3Transform = new Transform3d(new Pose3d(), new Pose3d(robotCenterToLimelight3));


    public static final double maxValidVelcity = 2.0; // m/s - ignoring vision data abve this velocity
    public static final double maxValidAngleDiff = 5.0; // degrees - ignoring vision data if vision heading is off by more than this value
    public static final double maxDistanceOfCameraFromAprilTag = 4; // meters - ignoring vision data if apriltag is farther than this value
  }
}
