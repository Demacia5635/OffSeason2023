package frc.robot.utils.VisionUtils;

import java.util.Arrays;
import java.util.Comparator;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
// import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis;

public class Vision {
    
    private static final double maxValidVelocity = 2.0; // m/s - vision data will be ingored when the robot is at a higher speed.
    private static final double maxValidAngleDiff = 10;  // degrees - vision data will be ingored if the diffrence between vision and motors is higher than this.
    
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

    SwerveDrivePoseEstimator poseEstimator;
    Chassis chassis;
    VisionData[] buf = new VisionData[3];
    int lastData = -1;
    double lastUpdateTime;

    public Pose2d getPose(){

    }

    int next() {
        return (lastData+1)%buf.length;
    }

    public Vision(Chassis chassis, SwerveDrivePoseEstimator estimator) {
        this.chassis = chassis;
        poseEstimator = estimator;
        for(int i = 0; i < buf.length; i++) {
            buf[i] = new VisionData(null, 0);
        }
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
    private VisionData median(VisionData[] visionDataArr){
        Arrays.sort(visionDataArr,comperator);
        return visionDataArr[visionDataArr.length/2];
    }
}
