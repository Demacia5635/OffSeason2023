
package frc.robot.Util;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RoundedPoint{
    double radius;
    Translation2d aPoint;
    Translation2d bPoint;
    Translation2d cPoint;
    Translation2d vectorAtoB;
    Translation2d vectorBtoC;
    public RoundedPoint(double radius, Translation2d aPoint, Translation2d bPoint, Translation2d cPoint){
        this.radius = radius;
        this.aPoint = aPoint;
        this.bPoint = bPoint;
        this.cPoint = cPoint;
        vectorAtoB = bPoint.minus(aPoint);
        vectorBtoC = cPoint.minus(bPoint);
    }
    private double getCornerAngle(){
        double bigAngle = vectorAtoB.times(-1).getAngle().getDegrees();
        double smallAngle = vectorBtoC.getAngle().getDegrees();
        return smallAngle - bigAngle;
    }
    private double getDirection(){
        double direction = (getCornerAngle() / 2) + vectorAtoB.times(-1).getAngle().getDegrees();
        return direction;
    }
    private Translation2d getAngleVector(){
        Translation2d vector = new Translation2d(1, new Rotation2d(Math.toRadians(getDirection())));
        return vector;
    }
    private double getLength(){
        double length = radius / Math.sin(Math.toRadians(Math.abs(getCornerAngle() / 2)));
        System.out.println(length);
        return length;
    }
    private Translation2d getVector(){
        return getAngleVector().times(getLength());
    }
    public Translation2d getCenterCircle(){
        return getVector().plus(bPoint);
    }



    
}
