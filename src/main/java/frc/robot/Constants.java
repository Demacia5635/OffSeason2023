package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static final double CYCLE_DT = 0.02;

 public static final double aprilTagValidTime = 2;
  

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



  public static final class VisionConstants {

    public static final String photonCamera1Name = "OV5647";
    public static final PhotonCamera photonCamera1 = new PhotonCamera(photonCamera1Name);
    public static final String photonCamera2Name = "";
    public static final int photonCameraNum1 = 1;
    public static final int photonCameraNum2 = 2;
    public static final Pose2d robotCenterToCamera = new Pose2d(new Translation2d(0.23, -0.8),
        Rotation2d.fromDegrees(-35));

    public static final Transform3d robotCenterToCameraTransform = new Transform3d(new Pose3d(), new Pose3d(robotCenterToCamera));

    public static final double maxValidVelcity = 2.0; // m/s - ignoring vision data abve this velocity
    public static final double maxValidAngleDiff = 10.0; // degrees - ignoring vision data if vision heading is off by
                                                         // more than this value
    public static final double maxDistanceOfCameraFromAprilTag = 4; // meters - ignoring vision data if apriltag is
                                                                    // farther than this value
  }

  public static final int CONTROLLER_PORT = 0;

  public static class ChassisConstants {
    public final static SwerveModuleConstants MODULE_FRONT_LEFT = new SwerveModuleConstants(
      7, 8, 11,
      new Translation2d(0.26515, 0.2215),
      35.068359375
    );
    public final static SwerveModuleConstants MODULE_FRONT_RIGHT = new SwerveModuleConstants(
      5, 6, 13,
      new Translation2d(0.26515, -0.2215),
      18.984375
    );
    public final static SwerveModuleConstants MODULE_BACK_LEFT = new SwerveModuleConstants(
      1, 2, 10,
      new Translation2d(-0.25451, 0.2065),
      227.900390625
    );
    public final static SwerveModuleConstants MODULE_BACK_RIGHT = new SwerveModuleConstants(
      3, 4, 12,
      new Translation2d(-0.26515, -0.2215),
      37.001953125
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

    public static final double PULSES_PER_METER = 52226.56641604010025062656641604;
    public static final double PULSES_PER_DEGREE = 72.817777777777777777777777777779;

    public static class SwerveModuleConstants {
      public static final double MOVE_KP = 0.007;
      public static final double MOVE_KI = 0;
      public static final double MOVE_KD = 0.012;
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

  public static class LedConstants {
    public static final int LED_ID = 0;
    public static final int LED_COUNT = 171;
  }
}
