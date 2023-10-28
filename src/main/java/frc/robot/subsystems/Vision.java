package frc.robot.subsystems;

import java.io.IOException;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.VisionUtils.SwerveDrivePoseEstimator;

import static frc.robot.Constants.VisionConstants.*;

public class Vision extends SubsystemBase{
    
    
    PhotonCamera camera;
    SwerveDrivePoseEstimator poseEstimator;
    PhotonPoseEstimator photonPoseEstimator;
    Chassis chassis;
    VisionData[] buf = new VisionData[3];
    int lastData = -1;
    double lastUpdateTime;

    public Vision(Chassis chassis, SwerveDrivePoseEstimator estimator) {
        camera = new PhotonCamera(photonCameraName);
        this.chassis = chassis;
        for(int i = 0; i < buf.length; i++) {
            buf[i] = new VisionData(null, 0);
        }
    }
    
    public void updateRobotPose(){
        double time = getTime();
        {
            if(validBuf(time)) {
                VisionData vData = median(buf);
                if(vData != null && vData.pose != null) {
                    poseEstimator.addVisionMeasurement(vData.pose, getTime() - vData.timeStamp);
                    lastUpdateTime = time;
                    time = vData.timeStamp;
                    for(VisionData vd : buf) {
                        vd.recalc(time);
                    }
                }
            }
            else{
                VisionData dataBeforelastData = buf[lastData-1];
                VisionData dataLastData = buf[lastData];
                buf[lastData - 2] = dataBeforelastData;
                buf[lastData - 1] = dataLastData;
            }
        }
    }

    private void updateVisionData() {
        if(chassis.getVelocity() <= maxValidVelcity ) {
            var latestResult = camera.getLatestResult();
            var bestTarget = latestResult.getBestTarget();
            int targetID = bestTarget.getFiducialId();
            lastData = next();
            Pose2d aprilTagToFieldPose2d;
            try {
                aprilTagToFieldPose2d = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField().getTagPose(targetID).get().toPose2d();
            } catch (IOException e) {
                return;
            }
            Pose2d cameraToAprilTagPose2d = new Pose2d(bestTarget.getBestCameraToTarget().getTranslation().toTranslation2d(), bestTarget.getBestCameraToTarget().getRotation().toRotation2d());
            Pose2d visionRobotToFieldPose = new Pose2d(); // !
            VisionData newVisionData = new VisionData(visionRobotToFieldPose, latestResult.getTimestampSeconds() - (latestResult.getLatencyMillis()/1000));
            if(newVisionData != null && newVisionData.pose != null){
                if((newVisionData.pose).getTranslation().getDistance(aprilTagToFieldPose2d.getTranslation()) > maxDistanceOfCameraFromAprilTag)
                    return;
                buf[lastData] = newVisionData;
            }
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        updateVisionData();
        updateRobotPose();
    }

    int next() {
        return (lastData+1)%buf.length;
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
        for(VisionData vData : buf) {
            if(vData.timeStamp < minTime) {
                return false;
            }
        }
        return true;
    }
    private VisionData median(VisionData[] visionDataArr){
        Arrays.sort(visionDataArr,comperator);
        return visionDataArr[visionDataArr.length/2];
    }

    public double lastUpdateLatency() {
        return getTime() - lastUpdateTime;
    }

    public boolean validVisionPosition() {
        return lastUpdateLatency() < 1;
    }

    class VisionData{
    
        private Pose2d pose;
        private double timeStamp;
        private double diffrence; // difference than odometry

        VisionData(Pose2d pose, double timeStamp){
            this.pose = pose;
            this.timeStamp = timeStamp;
            if(timeStamp > 0) {
                setDiffrence();
            } else {
                clear();
            }
        } 

        void recalc(double time) {
            // for newer data - recalc the twsit
            if(timeStamp > time) {
                setDiffrence();
            } else {
                clear();
            }
        }

        void setDiffrence() {
            Pose2d poseSample = poseEstimator.getSample(timeStamp);
            if(poseSample != null && Math.abs(poseSample.getRotation().minus(pose.getRotation()).getDegrees()) < maxValidAngleDiff) {
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
