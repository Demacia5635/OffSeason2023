
package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RoundedPoint extends Pose2d {
    double radius;
    Translation2d point;

    public RoundedPoint(double radius, Translation2d point){
        this.radius = radius;
        this.point = point;
    }

    

}
