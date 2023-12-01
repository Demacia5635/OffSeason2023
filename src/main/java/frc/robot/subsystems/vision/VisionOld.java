package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Arrays;
import java.util.Comparator;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.vision.utils.SwerveDrivePoseEstimator;

import static frc.robot.Constants.VisionConstants.*;

public class VisionOld extends SubsystemBase {
    Field2d checkField2dPoseEstimator;
    Field2d checkField2dVision;
    PhotonCamera camera1;
    PhotonCamera camera2;
    SwerveDrivePoseEstimator poseEstimator;
    PhotonPoseEstimator photonPoseEstimator;
    Chassis chassis;
    VisionData[] buf = new VisionData[3];
    int lastData = -1;
    double lastUpdateTime;
    boolean firstRun;

    public VisionOld(Chassis chassis, SwerveDrivePoseEstimator estimator) {
        camera1 = new PhotonCamera(photonCamera1Name);
        //camera2 = new PhotonCamera(photonCamera2Name);
        this.chassis = chassis;
        this.poseEstimator = estimator;
        checkField2dVision = new Field2d();
        checkField2dPoseEstimator = new Field2d();
        firstRun = true;
        for (int i = 0; i < buf.length; i++) {
            buf[i] = new VisionData(null, 0);
        }
        SmartDashboard.putData("field check pose estimator", checkField2dPoseEstimator);
        SmartDashboard.putData("field check vision", checkField2dVision);
        ;
    }
    
    public void updateRobotPose() {
        double time = getTime();
        {
            if (validBuf(time)) {
                VisionData vData = median(buf);
                if (vData != null && vData.pose != null) {
                    System.out.println(vData.pose);
                    //poseEstimator.addVisionMeasurement(vData.pose, getTime() - vData.timeStamp);
                    //checkField2dPoseEstimator.setRobotPose(poseEstimator.getEstimatedPosition());
                    checkField2dVision.setRobotPose(vData.pose);
                    lastUpdateTime = time;
                    time = vData.timeStamp;
                    for (VisionData vd : buf) {
                        vd.recalc(time);
                    }
                }
            } 
            // else {
            //     VisionData dataBeforelastData = buf[lastData - 1];
            //     VisionData dataLastData = buf[lastData];
            //     buf[lastData - 2] = dataBeforelastData;
            //     buf[lastData - 1] = dataLastData;
            // }
        }
    }

    // private void getNewVisionDataFromCameraX(int x) {
    //     // Choosing Camera.
    //     PhotonCamera camera;
    //     if (x == photonCameraNum1)
    //         camera = camera1;
    //     else
    //         camera = camera2;

    //     if (chassis.getVelocity().getNorm() <= maxValidVelcity) {
    //         var latestResult = camera.getLatestResult();
    //         if (latestResult.hasTargets()) {
    //             var bestTarget = latestResult.getBestTarget();
    //             if (bestTarget != null) {
    //                 int targetID = bestTarget.getFiducialId();
    //                 lastData = next();
    //                 Pose2d aprilTagToFieldPose2d;
    //                 try {
    //                     aprilTagToFieldPose2d = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField().getTagPose(targetID)
    //                             .get().toPose2d();
    //                 } catch (IOException e) {
    //                     return;
    //                 }
    //                 Pose2d cameraToAprilTagPose2d = new Pose2d(
    //                         bestTarget.getBestCameraToTarget().getTranslation().toTranslation2d(),
    //                         bestTarget.getBestCameraToTarget().getRotation().toRotation2d());
    //                 System.out.println("camera to april tag pose,jlvkvkj before - " + cameraToAprilTagPose2d);
    //                 cameraToAprilTagPose2d = new Pose2d(cameraToAprilTagPose2d.getTranslation().rotateBy(
    //                     aprilTagToFieldPose2d.getRotation().minus(cameraToAprilTagPose2d.getRotation())), cameraToAprilTagPose2d.getRotation());   
    //                 System.out.println("april tag pose - " + aprilTagToFieldPose2d);
    //                 System.out.println("camera to april tag pose after - " + cameraToAprilTagPose2d);
    //                 Translation2d visionRobotToFieldTranslation2d = aprilTagToFieldPose2d.getTranslation().minus(cameraToRobotCenter.getTranslation()).plus(cameraToAprilTagPose2d.getTranslation());
    //                 Rotation2d visionRobotToFieldRotation2d = cameraToAprilTagPose2d.getRotation().plus(cameraToRobotCenter.getRotation()).minus(Rotation2d.fromDegrees(180));
    //                 VisionData newVisionData = new VisionData(new Pose2d(visionRobotToFieldTranslation2d, visionRobotToFieldRotation2d), latestResult.getTimestampSeconds() - (latestResult.getLatencyMillis()/1000));
    //                 if (newVisionData != null && newVisionData.pose != null) {
    //                     if ((newVisionData.pose).getTranslation()
    //                             .getDistance(aprilTagToFieldPose2d.getTranslation()) > maxDistanceOfCameraFromAprilTag)
    //                         return;
    //                     buf[lastData] = newVisionData;
    //                 }
    //             }
    //         }
    //     }
    // }


    @Override
    public void periodic() {
        super.periodic();
        // getNewVisionDataFromCameraX(photonCameraNum1);
        //getNewVisionDataFromCameraX(photonCameraNum2);
        updateRobotPose();
        
        
        // checkFunc(photonCameraNum1);
        // checkUpdateRobotPose();
    }

    int next() {
        return (lastData + 1) % buf.length;
    }

    public double getTime() {
        return Timer.getFPGATimestamp();
    }

    Comparator<VisionData> comperator = new Comparator<VisionData>() {
        @Override
        public int compare(VisionData data0, VisionData data1) {
            return Double.compare(data0.diffrence, data1.diffrence);
        }
    };

    private boolean validBuf(double time) {
        double minTime = time - 1.2;
        for (VisionData vData : buf) {
            if (vData.timeStamp < minTime) {
                return false;
            }
        }
        return true;
    }

    private VisionData median(VisionData[] visionDataArr) {
        Arrays.sort(visionDataArr, comperator);
        return visionDataArr[visionDataArr.length / 2];
    }

    public double lastUpdateLatency() {
        return getTime() - lastUpdateTime;
    }

    public boolean validVisionPosition() {
        return lastUpdateLatency() < 1;
    }

    class VisionData {

        private Pose2d pose;
        private double timeStamp;
        private double diffrence; // difference than odometry

        VisionData(Pose2d pose, double timeStamp) {
            this.pose = pose;
            this.timeStamp = timeStamp;
            if (timeStamp > 0) {
                // setDiffrence();
            } else {
                clear();
            }
        }

        void recalc(double time) {
            // for newer data - recalc the twsit
            if (timeStamp > time) {
                // setDiffrence();
            } else {
                clear();
            }
        }

        void setDiffrence() {
            Pose2d poseSample = poseEstimator.getSample(timeStamp);
            if (poseSample != null
                    && Math.abs(poseSample.getRotation().minus(pose.getRotation()).getDegrees()) < maxValidAngleDiff) {
                diffrence = poseSample.getTranslation().getDistance(pose.getTranslation());
            } else {
                clear();
            }
        }

        void clear() {
            diffrence = -1;
            pose = null;
            timeStamp = 0;
        }
    }

}
