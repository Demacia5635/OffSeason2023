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

    public static final double MOVE_KP = 0.2;
    public static final double MOVE_KI = 0;
    public static final double MOVE_KD = 0;
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

  public static final class VisionConstants {

    public static final String photonCamera1Name = "Limelight2";
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
}
