package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Arrays;
import java.util.Comparator;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.vision.utils.SwerveDrivePoseEstimator;

import static frc.robot.Constants.VisionConstants.*;

public class Vision extends SubsystemBase {
    Field2d poseEstimatorField;
    Field2d visionField;
    PhotonCamera camera1;
    PhotonCamera camera2;
    SwerveDrivePoseEstimator poseEstimator;
    PhotonPoseEstimator photonPoseEstimator;
    Chassis chassis;
    VisionData[] buf = new VisionData[3];
    int lastData = -1;
    double lastUpdateTime;
    boolean firstRun;

    public Vision(Chassis chassis, SwerveDrivePoseEstimator estimator) {
        camera1 = photonCamera1;
        // camera1 = new PhotonCamera(photonCamera1Name);
        //camera2 = new PhotonCamera(photonCamera2Name);
        this.chassis = chassis;
        this.poseEstimator = estimator;
        try {
            photonPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile), PoseStrategy.AVERAGE_BEST_TARGETS, camera1, robotCenterToCameraTransform);
        } catch (IOException e) {
            e.printStackTrace();
        }        
        visionField = new Field2d();
        poseEstimatorField = new Field2d();
        firstRun = true;
        for (int i = 0; i < buf.length; i++) {
            buf[i] = new VisionData(null, 0);
        }
        SmartDashboard.putData("field check pose estimator", poseEstimatorField);
        SmartDashboard.putData("field check vision", visionField);
        ;
    }
    
    public void updateRobotPose() {
        double time = getTime();
        {
            if (validBuf(time)) {
                VisionData vData = median(buf);
                if (vData != null && vData.pose != null) {
                    // System.out.println(vData.pose);
                    poseEstimator.addVisionMeasurement(vData.pose, vData.timeStamp);
                    poseEstimatorField.setRobotPose(poseEstimator.getEstimatedPosition());
                    visionField.setRobotPose(vData.pose);
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

    private void getNewVisionDataFromCamera() {
        if (chassis.getVelocity().getNorm() <= maxValidVelcity) {
            var PhotonUpdate = photonPoseEstimator.update();
            if(PhotonUpdate != null && PhotonUpdate.get() != null){
                var estimatedRobotPose = PhotonUpdate.get();
                var estimatedPose = estimatedRobotPose.estimatedPose;
                if(estimatedRobotPose != null){
                    lastData = next();
                    VisionData newVisionData = new VisionData(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
                    if (newVisionData != null && newVisionData.pose != null) {
                        // if ((newVisionData.pose).getTranslation()
                        //         .getDistance(estimatedRobotPose.getTranslation()) > maxDistanceOfCameraFromAprilTag)
                        //     return; removed this filter because they use multiple april tags at the same time
                        buf[lastData] = newVisionData;
                    }   
                }
            }
        }
    }


    @Override
    public void periodic() {
        super.periodic();
        getNewVisionDataFromCamera();
        //getNewVisionDataFromCameraX(photonCameraNum2);
        updateRobotPose();
        
        
        
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
