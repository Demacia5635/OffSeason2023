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

    public final static double[] KS = {5.16955116, 24.02165666, 2.162370422, -0.061468493};
    public final static double[] KA = {-1.202533577, -58.75892648, -0.923979363, 0.125707695};
    public final static double[] Kalpha = {-15.69829677, -13.97516149, -4.939897152, 0.200128503};
    public final static double[] KV = {0.005323938, 0.005160888, 0.004178716, 0.004579808};
    public final static double[] Ksin = {0.180088628, -0.556040007, 0.047218921, -0.001551788};
    public final static double[] Kcos = {-0.004043448, -0.003561683, 0.005148196, -0.001570722};
    public final static double[] Kcossin = {1.287228076, 64.46158937, 0.883667392, -0.172153887};

  }
  public static class OperatorConstants {
	public static final int kDriverControllerPort = 0;
  }
}
