package frc.robot.utils.VisionUtils;

import java.util.Arrays;
import java.util.Comparator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
// import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class Vision {
    
    
    
    SwerveDrivePoseEstimator poseEstimator;
    Chassis chassis;
    VisionData[] buf = new VisionData[3];
    int lastData = -1;
    double lastUpdateTime;

    public Vision(Chassis chassis, SwerveDrivePoseEstimator estimator) {
        this.chassis = chassis;
        for(int i = 0; i < buf.length; i++) {
            buf[i] = new VisionData(null, 0);
        }
    }
    
    public Pair<Pose2d, Double> getPose(){
        if(chassis.getVelocity() > Constants.VisionConstants.maxValidVelocity ) {
            return null;
        }
        double time = getTime();
        lastData = next();
        VisionData newVisionData = getVisionData();
        if(newVisionData != null && newVisionData.pose != null){
            buf[lastData] = getVisionData();
            if(validBuf(time)) {
                VisionData vData = median(buf);
                if(vData != null && vData.pose != null) {
                    Pair<Pose2d, Double> poseAndLatency = new Pair<Pose2d, Double>(vData.pose, getTime()-vData.timeStamp);
                    lastUpdateTime = time;
                    time = vData.timeStamp;
                    for(VisionData vd : buf) {
                        vd.recalc(time);
                    }
                    return poseAndLatency;
                }
            }
            else{
                VisionData dataBeforelastData = buf[lastData-1];
                VisionData dataLastData = buf[lastData];
                buf[lastData - 2] = dataBeforelastData;
                buf[lastData - 1] = dataLastData;
            }
        }
        return null;
    }

    private VisionData getVisionData() {
        return null;
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
            if(poseSample != null && Math.abs(poseSample.getRotation().minus(pose.getRotation()).getDegrees()) < Constants.VisionConstants.maxValidAngleDiff) {
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
