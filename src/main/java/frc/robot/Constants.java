// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ArmConstants{
    public final static int motorID = 30;
    public final static int DigitalInputID = 0;

    public final static double pulesPerRotation = 2048;
    public final static double gearRatio = 180;
    public final static double pulsePerAngle = pulesPerRotation*gearRatio/360;

    public final static double KP = 3.596;
    public final static double KI = 0;
    public final static double KD = 0;

    public final static double KA = 0;
    public final static double KV = 0;
    public final static double KS = 0;
    public final static double Kalpha = 0;
    public final static double Ksin = 0;
    public final static double Kcos = 0;
    public final static double Kcossin = 0;

  }
  public static class OperatorConstants {
	public static final int kDriverControllerPort = 0;
  }
}
