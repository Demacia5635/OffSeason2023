package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static final double CYCLE_DT = 0.02;
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

    public static final double VELOCITY = 2;
    public static final double ACCELERATION = 4;
    public static final double ANGULAR_VELOCITY = 180;
    public static final double ANGULAR_ACCELERATION = 360;

    public static final double PULSES_PER_METER = 52226.56641604010025062656641604;
    public static final double PULSES_PER_DEGREE = 72.817777777777777777777777777779;

    public static class SwerveModuleConstants {
      public static final double MOVE_KP = 0.002;
      public static final double MOVE_KI = 0.0001;
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

  public static class LedConstants {
    public static final int LED_ID = 0;
    public static final int LED_COUNT = 171;
  }
}
