package frc.robot.subsystems.vision.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionData {

    public Pose2d pose;
    public double timeStamp;
    public double diffrence; // difference than odometry
    public Pose2d falsePose;
    public double falsetimeStamp;


    public VisionData(Pose2d pose, double timeStamp) {
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