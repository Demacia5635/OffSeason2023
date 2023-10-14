

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
 // private static double voltsPerMeter = 0.417;

  public static final int DriverControllerPort = 0;
  public static class SwerveChassisConstants {
    public static final SwerveModuleConstants frontLeft = new SwerveModuleConstants(new Translation2d(-1, 1));
    public static final SwerveModuleConstants frontRight = new SwerveModuleConstants(new Translation2d(1, 1));
    public static final SwerveModuleConstants backLeft = new SwerveModuleConstants(new Translation2d(-1, -1));
    public static final SwerveModuleConstants backRight = new SwerveModuleConstants(new Translation2d(1, -1));

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      frontLeft.modulePosition, frontRight.modulePosition,
      backLeft.modulePosition, backRight.modulePosition
    );
  }

  public static class SwerveModuleConstants {
    public final Translation2d modulePosition;

    public SwerveModuleConstants(Translation2d modulePosition) {
      this.modulePosition = modulePosition;
    }
  }

  public static final double countPerMeter = 1;
  public static final double degreesPerSecond = 270;
  public static final double pulsesPerDegree = 1;

  public static final double widthWheels = 0.58;
  public static final double cycleTime = 0.02;



  }

