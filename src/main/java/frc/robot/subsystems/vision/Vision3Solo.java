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

public class Vision3Solo extends SubsystemBase {
    //declaring fields for testing
    Field2d poseEstimatorField;
    Field2d visionField;
    Field2d visionField3;
    

    //declaring limelights
    PhotonCamera Limelight2;
    PhotonPoseEstimator photonPoseEstimatorForLimelight2;
    PhotonCamera Limelight3;
    PhotonPoseEstimator photonPoseEstimatorForLimelight3;
    
    // declaring poseEstimator chassis and buffers 
    SwerveDrivePoseEstimator poseEstimator;
    Chassis chassis;

    int lastData;
    double lastUpdateTime;
    boolean firstRun;


    public Vision3Solo(Chassis chassis, SwerveDrivePoseEstimator estimator) {
        this.chassis = chassis;
        this.poseEstimator = estimator;
        this.Limelight2 = new PhotonCamera(Limelight2Name);
        this.Limelight3 = new PhotonCamera(Limelight3Name);
        
        this.firstRun = true;

        //initializing photons pose estimators
        try {
            this.photonPoseEstimatorForLimelight2 = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile),
             PoseStrategy.AVERAGE_BEST_TARGETS, Limelight2, robotCenterToLimelight2Transform);

             this.photonPoseEstimatorForLimelight3 = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile),
             PoseStrategy.AVERAGE_BEST_TARGETS, Limelight3, robotCenterToLimelight3Transform);
        } catch (IOException e) {
            System.out.println("problem with photon pose estimators");
            e.printStackTrace();
        } 
        
       
        //initializing fields for testing
        this.visionField= new Field2d();   
        this.visionField3 = new Field2d();
        this.poseEstimatorField = new Field2d();
        
        //putting testing fields on shuffleboard
        SmartDashboard.putData("no Filter vision", visionField);
        SmartDashboard.putData("field check pose estimator", poseEstimatorField);
        
       

    }
    
    //takes the visions snapshots from the buffer and medians or avg it and add vision mesurements to pose estimator
    public void updateRobotPose() {
        PhotonPoseEstimator photonPoseEstimator = photonPoseEstimatorForLimelight2;
        var PhotonUpdate = photonPoseEstimator.update();
        if(PhotonUpdate != null){
            try {
                var estimatedRobotPose = PhotonUpdate.get();
                var estimatedPose = estimatedRobotPose.estimatedPose;
                if(estimatedRobotPose != null){
                    VisionData newVisionData = new VisionData(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
                    if(firstRun){
                        poseEstimator.resetPosition(chassis.getAngle(), chassis.getModulePositions(), newVisionData.getFalsePose());
                        firstRun = false;
                    }
                    if (newVisionData != null && newVisionData.getPose() != null) {
                        poseEstimator.addVisionMeasurement(newVisionData.pose, newVisionData.timeStamp);
                        poseEstimatorField.setRobotPose(poseEstimator.getEstimatedPosition());
                        visionField.setRobotPose(newVisionData.getPose());
                    }   
                }
            } catch (NoSuchElementException e) {
                //System.out.println("got exception at get new data eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee");
            }

        }
            
    }

   


    @Override
    public void periodic() {
        super.periodic();

      
        updateRobotPose();

        
        Pose2d visionPose = visionField.getRobotPose();
        SmartDashboard.putNumber("no filter X", visionPose.getX());
        SmartDashboard.putNumber("no filter Y", visionPose.getY());
        

    }
    //util

    public double getTime() {
        return Timer.getFPGATimestamp();
    }

    
    Comparator<VisionData> comperator = new Comparator<VisionData>() {
        @Override
        public int compare(VisionData data0, VisionData data1) {
            return Double.compare(data0.getDiffrence(), data1.getDiffrence());
        }
    };
    
    public VisionData median(VisionData[] visionDataArr) {
        Arrays.sort(visionDataArr, comperator);
        return visionDataArr[visionDataArr.length / 2];
    }

    

   

    /*
     * @param bla
     * @return bal2
     */

    public double lastUpdateLatency() {
        return getTime() - lastUpdateTime;
    }

    public boolean validVisionPosition() {
        return lastUpdateLatency() < 1;
    }
    
    
    //object to save vision data that includes a pose a timestamp and the difference at that moment from the odometrey
    class VisionData {
    
        private Pose2d pose;
        private double timeStamp;
        private double diffrence; // difference than odometry
        private Pose2d falsePose;
        private double falsetimeStamp;
    
    
        VisionData(Pose2d pose, double timeStamp) {
            this.pose = pose;
            this.timeStamp = timeStamp;
            this.falsePose = pose;
            this.falsetimeStamp = timeStamp;
            if (timeStamp < 0) {
                System.out.println("cleared at constructor, " + timeStamp);
                clear();
                
            } else {
                
                setDiffrence();
            }
        }
    
        //utils
    
        public void recalc(double time) {
            // for newer data - recalc the twsit
            if (timeStamp < time) {
                //setDiffrence();
            } else {
                System.out.println("cleared at recalc");
                clear();
            }
        }
    
        protected void setDiffrence() {
            Pose2d poseSample = poseEstimator.getSample(timeStamp);
            if (poseSample != null
                    /*&& Math.abs(poseSample.getRotation().minus(pose.getRotation()).getDegrees()) < maxValidAngleDiff*/) {
                diffrence = poseSample.getTranslation().getDistance(pose.getTranslation());
            } else {
                System.out.println("cleared on setDifference() func");
                clear();
            }
        }
    
        private void clear() {
            diffrence = -1;
            pose = null;
            timeStamp = 0;
            //System.out.println("Cleared.");
        }    
            
        //getters
    
        public Pose2d getPose() {
            return pose;
        }
    
        public double getTimeStamp() {
            return timeStamp;
        }
    
        public double getDiffrence() {
            return diffrence;
        }
    
        public Pose2d getFalsePose() {
            return falsePose;
        }
    
        public double getFalsetimeStamp() {
            return falsetimeStamp;
        }
    
    
        
    }

    //enum for choosing which limelight to use
    enum Limelight{
        Limelight2,
        Limelight3
    }
}
