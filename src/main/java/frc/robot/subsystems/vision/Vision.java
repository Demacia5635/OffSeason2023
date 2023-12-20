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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
    Field2d visionFieldavg3;
    Field2d visionFieldavg5;

    PhotonCamera Limelight2;
    PhotonPoseEstimator photonPoseEstimatorForLimelight2;

    PhotonCamera Limelight3;
    PhotonPoseEstimator photonPoseEstimatorForLimelight3;

    SwerveDrivePoseEstimator poseEstimator;
    Chassis chassis;
    VisionData[] buf3 = new VisionData[3];
    VisionData[] buf5 = new VisionData[5];

    int lastData = -1;
    int lastData5 = -1;
    double lastUpdateTime;
    double lastUpdateTime5;
    boolean firstRun;


    public Vision(Chassis chassis, SwerveDrivePoseEstimator estimator) {
        this.chassis = chassis;
        this.poseEstimator = estimator;
        
        Limelight2 = new PhotonCamera(Limelight2Name);
        Limelight3 = new PhotonCamera(Limelight3Name);
   
        try {
            photonPoseEstimatorForLimelight2 = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile),
             PoseStrategy.AVERAGE_BEST_TARGETS, Limelight2, robotCenterToLimelight2Transform);

            photonPoseEstimatorForLimelight3 = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile),
             PoseStrategy.AVERAGE_BEST_TARGETS, Limelight3, robotCenterToLimelight3Transform);
        } catch (IOException e) {
            System.out.println("problem with photon pose estimators");
            e.printStackTrace();
        } 
        
        //setting up buffers
        for (int i = 0; i < buf3.length; i++) {
            buf3[i] = new VisionData(null, 0);
        }
        for (int i = 0; i < buf5.length; i++) {
            buf5[i] = new VisionData(null, 0);
        }

        //fields for testing
        visionField= new Field2d();   
        visionField3 = new Field2d();
        visionField5 = new Field2d();
        visionFieldavg3 = new Field2d();
        visionFieldavg5 = new Field2d();


        poseEstimatorField = new Field2d();
        
        // SmartDashboard.putData("field check pose estimator", poseEstimatorField);
        SmartDashboard.putData("vision3", visionField3);
        SmartDashboard.putData("vision5", visionField5);
        
        SmartDashboard.putData("vision", visionField);
        
        SmartDashboard.putData("visionavg3", visionFieldavg3);
        SmartDashboard.putData("visionavg5", visionFieldavg5);

        this.firstRun = true;
    }
    
    public void updateRobotPose() {
        double time = getTime();
        {
            if (validBuf(time)) {
                VisionData vData = median(buf3);
                VisionData vDataAvg = avg(buf3);
                if (vData != null && vData.pose != null) {
                    //poseEstimator.addVisionMeasurement(vData.pose, vData.timeStamp);
                    //poseEstimatorField.setRobotPose(poseEstimator.getEstimatedPosition());
                    visionField3.setRobotPose(vData.pose);
                    visionFieldavg3.setRobotPose(vDataAvg.pose);
                    lastUpdateTime = time;
                    time = vData.timeStamp;
                    // for (VisionData vd : buf3) {
                    //     vd.recalc(time);
                    // }
                    
                }
            } 
        }
    }

    public void updateRobotPose5() {
        
        double time = getTime();
        if (validBuf5(time)) {
            VisionData vData5 = median(buf5);
            VisionData vDataAvg5 = avg(buf5);
            if (vData5 != null && vData5.pose != null) {
                //poseEstimator.addVisionMeasurement(vData5.pose, vData5.timeStamp);
                // poseEstimatorField.setRobotPose(poseEstimator.getEstimatedPosition());
                visionField5.setRobotPose(vData5.pose);
                visionFieldavg5.setRobotPose(vDataAvg5.pose);
                lastUpdateTime5 = time;
                time = vData5.timeStamp;
                // for (VisionData vd : buf5) {
                //     vd.recalc(time);
                // }
            }
        } 
    }

    private void getNewDataFromLimelightX(Limelight x) {
        //determines camera
        PhotonPoseEstimator photonPoseEstimator;
        if(x.equals(Limelight.Limelight2))
            photonPoseEstimator = photonPoseEstimatorForLimelight2;
        else
            photonPoseEstimator = photonPoseEstimatorForLimelight3;
        if (chassis.getVelocity().getNorm() <= maxValidVelcity) {
            var PhotonUpdate = photonPoseEstimator.update();
            if(PhotonUpdate != null){
                try {
                    var estimatedRobotPose = PhotonUpdate.get();
                    var estimatedPose = estimatedRobotPose.estimatedPose;
                    if(estimatedRobotPose != null){
                        VisionData newVisionData = new VisionData(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
                        if(firstRun){
                            SwerveModulePosition[] positions = new SwerveModulePosition[4];
                            for (int i = 0; i < positions.length; i++) {                            
                                positions[i] = new SwerveModulePosition();
                            }
                            SmartDashboard.putBoolean("isSwerveNull", positions[2] == null);
                            SmartDashboard.putNumber("pose temp", newVisionData.falsePose.getRotation().getDegrees());
                            poseEstimator.resetPosition(newVisionData.falsePose.getRotation(), positions, newVisionData.falsePose);
                            SmartDashboard.putNumber("pose temp after", poseEstimator.getEstimatedPosition().getRotation().getDegrees());

                            firstRun = false;
                        }
                        if (newVisionData != null && newVisionData.pose != null) {
                            // if ((newVisionData.pose).getTranslation()
                            //         .getDistance(estimatedRobotPose.getTranslation()) > maxDistanceOfCameraFromAprilTag)
                            //     return; removed this filter because they use multiple april tags at the same time
                            lastData = next();
                            lastData5 = next5(); 
                            buf3[lastData] = newVisionData;
                            buf5[lastData5] = newVisionData;

                            // System.out.println(newVisionData.timeStamp + ", in index:" + lastData5);
                            // System.out.println("---------------------------");
                            // for (int i = 0; i < buf5.length; i++) {
                            //     VisionData visionBill = buf5[i];
                            //     System.out.println(visionBill.timeStamp + ", in index:" + i);
                            // }
                            // System.out.println("---------------------------");
                            visionField.setRobotPose(newVisionData.pose);
                        }   
                    }
                } catch (NoSuchElementException e) {
                    //System.out.println("");
                }
            }
        }
    }


    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("pereiodic pose rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());

        getNewDataFromLimelightX(Limelight.Limelight2);
        getNewDataFromLimelightX(Limelight.Limelight3);

        //System.out.println(buf5[lastData5].timeStamp);
        updateRobotPose();
        updateRobotPose5();
        Pose2d visionPose = visionField.getRobotPose();
        SmartDashboard.putNumber("no filter X", visionPose.getX());
        SmartDashboard.putNumber("no filter Y", visionPose.getY());
        Pose2d vision3Pose = visionField3.getRobotPose();
        SmartDashboard.putNumber("buf 3 med X", vision3Pose.getX());
        SmartDashboard.putNumber("buf 3 med Y", vision3Pose.getY());
        Pose2d vision5Pose = visionField5.getRobotPose();
        SmartDashboard.putNumber("buf 5 med X", vision5Pose.getX());
        SmartDashboard.putNumber("buf 5 med Y", vision5Pose.getY());    

        Pose2d vision3avgPose = visionFieldavg3.getRobotPose();
        SmartDashboard.putNumber("buf 3 avg X", vision3avgPose.getX());
        SmartDashboard.putNumber("buf 3 avg Y", vision3avgPose.getY());
        Pose2d vision5avgPose = visionFieldavg5.getRobotPose();
        SmartDashboard.putNumber("buf 5 avg X", vision5avgPose.getX());
        SmartDashboard.putNumber("buf 5 avg Y", vision5avgPose.getY());    

    }
    //util
    Comparator<VisionData> comperator = new Comparator<VisionData>() {
        @Override
        public int compare(VisionData data0, VisionData data1) {
            return Double.compare(data0.diffrence, data1.diffrence);
        }
    };

    public double getTime() {
        return Timer.getFPGATimestamp();
    }

    public VisionData avg(VisionData[] visionDataArr) {

        double averageX = 0.0;
        double averageY = 0.0;
        double averageRotation = 0.0;
        double avarageTimeStamp = 0.0;

        for (VisionData vData : visionDataArr) {
            if(vData != null && vData.pose != null){
                Pose2d pose = vData.pose;
                averageX += pose.getTranslation().getX();
                averageY += pose.getTranslation().getY();
                averageRotation += pose.getRotation().getRadians();
                avarageTimeStamp += vData.timeStamp;
            }
        }

        averageX /= visionDataArr.length;
        averageY /= visionDataArr.length;
        averageRotation /= visionDataArr.length;
        avarageTimeStamp /= visionDataArr.length;
        // Create a new Pose2d with the calculated averages
        return new VisionData((new Pose2d(averageX, averageY, new Rotation2d(averageRotation))), avarageTimeStamp);
    }

    private VisionData median(VisionData[] visionDataArr) {
        Arrays.sort(visionDataArr, comperator);
        return visionDataArr[visionDataArr.length / 2];
    }


    // BUF 3
    int next() {
        return (lastData + 1) % buf3.length;
    }

    /*
     * @param bla
     * @return bal2
     */
    private boolean validBuf(double time) {
        double minTime = time - 1.2;
        for (VisionData vData : buf3) {
            if (vData.timeStamp < minTime) {
                return false;
            }
        }
        return true;
    }

    public double lastUpdateLatency() {
        return getTime() - lastUpdateTime;
    }

    public boolean validVisionPosition() {
        return lastUpdateLatency() < 1;
    }
    // BUF 5
    int next5() {
        return (lastData5 + 1) % buf5.length;
    }
    
    private boolean validBuf5(double time) {
        double minTime = time - 2.5;
        for (VisionData vData : buf5) {
            //System.out.println(vData.timeStamp);
            if (vData.timeStamp < minTime) {
                return false;
            }
        }
        return true;
    }
   
    public double lastUpdateLatency5() {
        return getTime() - lastUpdateTime5;
    }
    
    public boolean validVisionPosition5() {
        return lastUpdateLatency5() < 1;
    }

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
                
                //setDiffrence();
            }
        }

        protected void recalc(double time) {
            // for newer data - recalc the twsit
            if (timeStamp < time) {
                //setDiffrence();
            } else {
                System.out.println("cleared at recalc");
                clear();
            }
        }

        // protected void setDiffrence() {
        //     Pose2d poseSample = poseEstimator.getSample(timeStamp);
        //     if(poseSample != null)
        //        // System.out.println(/*poseSample.getRotation().getDegrees()*/ poseEstimator.getEstimatedPosition().getRotation().getDegrees() + " " + pose.getRotation().getDegrees());
        //     if (poseSample != null
        //             && Math.abs(poseSample.getRotation().minus(pose.getRotation()).getDegrees()) < maxValidAngleDiff) {
        //         diffrence = poseSample.getTranslation().getDistance(pose.getTranslation());
        //     } else {
        //         //System.out.println("cleared on setDifference() func");
        //         clear();
        //     }
        // }

        protected void clear() {
            diffrence = -1;
            pose = null;
            timeStamp = 0;
           // System.out.println("Cleared.");
        }
    }

    enum Limelight{
        Limelight2,
        Limelight3
    }
}
