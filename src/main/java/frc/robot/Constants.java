// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class VisionConstants {
    public static final NetworkTable LIMELIGHT_TABLE1 = NetworkTableInstance.getDefault().getTable("limelight-ii");
    public static final NetworkTable LIMELIGHT3_TABLE = NetworkTableInstance.getDefault().getTable("limelight-iii");

    public static final NetworkTableEntry HAS_TARGET_ENTRY = LIMELIGHT_TABLE1.getEntry("tv"); // double not boolean
    
    /**
     * An array of doubles with the following values:
     * <p>
     * [0] - meters from the corner of the blue alliance x axis
     * <p>
     * [1] - meters from the corner of the blue alliance y axis
     * <p>
     * [2] - meters from the the field carpet in the z axis
     * <p>
     * [3] - roll in degrees
     * <p>
     * [4] - pitch in degrees
     * <p>
     * [5] - yaw in degrees
     */
    public static final NetworkTableEntry ROBOT_POSE_ENTRY = LIMELIGHT_TABLE1.getEntry("botpose_wpiblue");

    /**
     * An array of doubles with the following values:
     * <p>
     * [0] - meters from the limelight to the april tag in the right direction
     * <p>
     * [1] - meters from the limelight to the april tag in the down direction
     * <p>
     * [2] - meters from the limelight to the april tag in the forward direction
     * <p>
     * [3] - pitch from the camera to the april tag in degrees
     * <p>
     * [4] - yaw from the camera to the april tag in degrees
     * <p>
     * [5] - roll from the camera to the april tag in degrees
     */
    public static final NetworkTableEntry CAMERA_TRANSLATION_ENTRY = LIMELIGHT_TABLE1
            .getEntry("targetpose_cameraspace");

    public static final double MAX_DISTANCE_FOR_LIMELIGHT = 3;

    public static final double maxValidVelocity = 2.0; // m/s - vision data will be ingored when the robot is at a higher speed.
    public static final double maxValidAngleDiff = 10;  // degrees - vision data will be ingored if the diffrence between vision and motors is higher than this.

    public static final double VISION_ANGLE_TOLERANCE = 5;

    public static final double LIMELIGHT2_YAW = 22.5;

    public static final double LIMELIGHT3_YAW = 32.2;

    public static final double MAX_SIDES_RATIO = 1.2;

    public static final double VISION_TX_LIMIT = 5;

    public static final double VISION_TA_LIMIT = 0.5;
  }
}
