package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Arrays;
import java.util.Comparator;
import java.util.NoSuchElementException;

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
    Field2d visionField3;
    Field2d visionField5;
    double[] visionFieldArr;
    double[] visionField3Arr;
    double[] visionField5Arr;
    PhotonCamera camera1;
    PhotonCamera camera2;
    SwerveDrivePoseEstimator poseEstimator;
    PhotonPoseEstimator photonPoseEstimator;
    Chassis chassis;
    VisionData[] buf3 = new VisionData[3];
    VisionData[] buf5 = new VisionData[5];
    int lastData = -1;
    int lastData5 = -1;
    double lastUpdateTime;
    double lastUpdateTime5;


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
        visionField= new Field2d();   
        visionField3 = new Field2d();
        visionField5 = new Field2d();
        poseEstimatorField = new Field2d();
        visionFieldArr = new double[2];
        visionField3Arr = new double[2];
        visionField5Arr = new double[2];
        for (int i = 0; i < buf3.length; i++) {
            buf3[i] = new VisionData(null, 0);
        }
        for (int i = 0; i < buf5.length; i++) {
            buf5[i] = new VisionData(null, 0);
        }
        // SmartDashboard.putData("field check pose estimator", poseEstimatorField);
        SmartDashboard.putData("vision3", visionField3);
        SmartDashboard.putData("vision5", visionField5);
        SmartDashboard.putData("vision", visionField);
        

        
    }
    
    public void updateRobotPose() {
        double time = getTime();
        {
            if (validBuf(time)) {
                VisionData vData = median(buf3);
                if (vData != null && vData.pose != null) {
                    // System.out.println(vData.pose);
                    poseEstimator.addVisionMeasurement(vData.pose, vData.timeStamp);
                    poseEstimatorField.setRobotPose(poseEstimator.getEstimatedPosition());
                    visionField3.setRobotPose(vData.pose);
                    lastUpdateTime = time;
                    time = vData.timeStamp;
                    for (VisionData vd : buf3) {
                        vd.recalc(time);
                    }
                }
            } 
        }
    }

    public void updateRobotPose5() {
        double time = getTime();
        if (validBuf5(time)) {
            VisionData vData5 = median(buf5);
            if (vData5 != null && vData5.pose != null) {
                // System.out.println(vData.pose);
                // poseEstimator.addVisionMeasurement(vData5.pose, vData5.timeStamp);
                // poseEstimatorField.setRobotPose(poseEstimator.getEstimatedPosition());
                visionField5.setRobotPose(vData5.pose);
                lastUpdateTime5 = time;
                time = vData5.timeStamp;
                for (VisionData vd : buf5) {
                    vd.recalc(time);
                }
            }
        } 
    }

    private void getNewVisionDataFromCamera() {
        if (chassis.getVelocity().getNorm() <= maxValidVelcity) {
            var PhotonUpdate = photonPoseEstimator.update();
            if(PhotonUpdate != null){
                try {
                    var estimatedRobotPose = PhotonUpdate.get();
                    var estimatedPose = estimatedRobotPose.estimatedPose;
                    if(estimatedRobotPose != null){
                        lastData = next();
                        lastData5 = next5();
                        VisionData newVisionData = new VisionData(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
                        if (newVisionData != null && newVisionData.pose != null) {
                            // if ((newVisionData.pose).getTranslation()
                            //         .getDistance(estimatedRobotPose.getTranslation()) > maxDistanceOfCameraFromAprilTag)
                            //     return; removed this filter because they use multiple april tags at the same time
                            buf3[lastData] = newVisionData;
                            buf5[lastData5] = newVisionData;
                            System.out.println("-----------------------------");
                            System.out.println(lastData5 + " : " + buf5[lastData5].timeStamp);
                            for(int i = 0; i < buf5.length; i++) {
                                System.out.println(i + " : " + buf5[i].timeStamp);
                                
                            }
                            System.out.println(validBuf5(getTime()));
                            visionField.setRobotPose(newVisionData.pose);
                        }   
                    }
                } catch (NoSuchElementException e) {
                    // TODO: handle exception
                }
            }
        }
    }


    @Override
    public void periodic() {
        super.periodic();
        getNewVisionDataFromCamera();
        updateRobotPose();
        updateRobotPose5();
        Pose2d visionPose = visionField.getRobotPose();
        SmartDashboard.putNumber("vision X", visionPose.getX());
        SmartDashboard.putNumber("vision Y", visionPose.getY());
        Pose2d vision3Pose = visionField3.getRobotPose();
        SmartDashboard.putNumber("vision3 X", vision3Pose.getX());
        SmartDashboard.putNumber("vision3 Y", vision3Pose.getY());
        Pose2d vision5Pose = visionField5.getRobotPose();
        SmartDashboard.putNumber("vision5 X", vision5Pose.getX());
        SmartDashboard.putNumber("vision5 Y", vision5Pose.getY());    
            
    }

    int next() {
        return (lastData + 1) % buf3.length;
    }
    int next5() {
        return (lastData5 + 1) % buf5.length;
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
        for (VisionData vData : buf3) {
            if (vData.timeStamp < minTime) {
                return false;
            }
        }
        return true;
    }
    private boolean validBuf5(double time) {
        double minTime = time - 2;
        for (VisionData vData : buf5) {
            System.out.println(vData.timeStamp);
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
    private VisionData median5(VisionData[] visionDataArr) {
        Arrays.sort(visionDataArr, comperator);
        return visionDataArr[3];
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
