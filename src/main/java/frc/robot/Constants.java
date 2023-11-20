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
    /*
     * motor id of the arm
     * and the id of the limit swtich to the arm
     */
    public final static int motorID = 30;
    public final static int DigitalInputID = 0;

    /*
     * get the pules per angle through math
     * use as the get angle and angelar velocity
     */
    private final static double pulesPerRotation = 2048;
    private final static double gearRatio = 180;
    public final static double pulsePerAngle = pulesPerRotation*gearRatio/360;


    /* the four numbers in each array is for every state of the arm
     * 
     * 0 = the arm is going forward and less then angle 43
     * 1 = the arm is going forward and more then angle 43
     * 2 = the arm is going backward and more then angle 43
     * 3 = the arm is going backward and less then angle 43
     */
    public final static double[] KS = {1.287228076, 0.004391115, 0.009169107, -0.901637521};
    public final static double[] KV = {0.005323938, -0.003403798, 0.005036478, 0.005099339};
    public final static double[] KA = {-0.004043448, 0.000252797, 0.003954434, -0.002012317};
    public final static double[] Kalpha = {0.180088628, 0.005150217, 0.004382342, -0.220649252};
    public final static double[] Ksin = {-15.69829677, 0, 0, 18.36092613};
    public final static double[] Kcos = {-1.202533577, 0, 0, 0.870858412};
    public final static double[] Kcossin = {5.16955116, 0, 0, -5.614001788};
    
  }
  public static class OperatorConstants {
	public static final int kDriverControllerPort = 0;
  }
}
