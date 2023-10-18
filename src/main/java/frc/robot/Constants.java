package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static final double CYCLE_DT = 0.02;

  public static class ChassisConstants {
    public final static SwerveModuleConstants MODULE_FRONT_LEFT = new SwerveModuleConstants(
      0, 0, 0, 0, 0, 0,
      new Translation2d(-1, 1)
    );
    public final static SwerveModuleConstants MODULE_FRONT_RIGHT = new SwerveModuleConstants(
      0, 0, 0, 0, 0, 0,
      new Translation2d(1, 1)
    );
    public final static SwerveModuleConstants MODULE_BACK_LEFT = new SwerveModuleConstants(
      0, 0, 0, 0, 0, 0,
      new Translation2d(-1, -1)
    );
    public final static SwerveModuleConstants MODULE_BACK_RIGHT = new SwerveModuleConstants(
      0, 0, 0, 0, 0, 0,
      new Translation2d(1, -1)
    );
    public static final int GYRO_ID = 0;

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

    public static final double MOVE_KP = 1;
    public static final double MOVE_KI = 0;
    public static final double MOVE_KD = 0;
    public static final double ANGLE_KP = 1;
    public static final double ANGLE_KI = 0;
    public static final double ANGLE_KD = 0;

    public static final int PULSES_PER_METER = 1;
    public static final int PULSES_PER_DEGREE = 1;

    public static class SwerveModuleConstants {
      public final int moveMotorId;
      public final int angleMotorId;
      public final int absoluteEncoderId;
      public final int kS;
      public final int kV;
      public final int kA;
      public final Translation2d moduleTranslationOffset;

      public SwerveModuleConstants(int moveMotorId, int angleMotorId, int absoluteEncoderId, int kS, int kV, int kA, Translation2d moduleTranslationOffset) {
        this.moveMotorId = moveMotorId;
        this.angleMotorId = angleMotorId;
        this.absoluteEncoderId = absoluteEncoderId;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.moduleTranslationOffset = moduleTranslationOffset;
      }
    }
  }
}
