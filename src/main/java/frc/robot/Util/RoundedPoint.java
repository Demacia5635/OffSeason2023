
package frc.robot.Util;
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

    
    //work
    private double getCornerAngle(){
        double dotProduct = (vectorAtoB.getX() * vectorBtoC.getX()) + (vectorAtoB.getY() * vectorBtoC.getY());
        double angle = Math.acos(dotProduct / (vectorAtoB.getNorm() * vectorBtoC.getNorm()));
        return Math.toDegrees(angle);
    }

    //work
    private double getDirection(){
        double direction = vectorAtoB.times(-1).getAngle().getDegrees() + (getCornerAngle() / 2);
        return direction;
    }
    public double getCrossAngleLength(){
        double length = Math.pow((Math.sin(getCornerAngle() / 2)), -1) * radius;
        System.out.println(length);
        return length;
    }

    public Translation2d createCornerVector(){
        Translation2d vector = new Translation2d(Math.cos(getDirection()), Math.sin(getDirection()));
        return vector.times(getCrossAngleLength());
    }
    
}
