
package frc.robot.Util;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RoundedPoint{
    double smoothnes = 10;
    double radius;
    Translation2d aPoint;
    Translation2d bPoint;
    Translation2d cPoint;
    Translation2d vectorAtoB;
    Translation2d vectorBtoC;
    double direction;

    public RoundedPoint(double radius, Translation2d aPoint, Translation2d bPoint, Translation2d cPoint){
        this.radius = radius;
        this.aPoint = aPoint;
        this.bPoint = bPoint;
        this.cPoint = cPoint;
        vectorAtoB = bPoint.minus(aPoint);
        vectorBtoC = cPoint.minus(bPoint);

        direction = (cornerAngle() / 2) + vectorAtoB.times(-1).getAngle().getDegrees();
    }
    private double cornerAngle(){
        double bigAngle = vectorAtoB.times(-1).getAngle().getDegrees();
        double smallAngle = vectorBtoC.getAngle().getDegrees();
        return smallAngle - bigAngle;
    }

    private Translation2d getAngleVector(){
        Translation2d vector = new Translation2d(1, new Rotation2d(Math.toRadians(direction)));
        return vector;
    }
    private double getLength(){
        double length = radius / Math.sin(Math.toRadians(Math.abs(cornerAngle() / 2)));
        System.out.println(length);
        return length;
    }
    private Translation2d getCenterCircle(){
        Translation2d vector = getAngleVector().times(getLength());
        return vector.plus(bPoint);
    }
    public Translation2d startRange()
    {
        return new Translation2d(radius, new Rotation2d(
            Math.toRadians(vectorAtoB.times(-1).getAngle().getDegrees() - (90 *Math.signum(cornerAngle())))
        ));
    }

    public Translation2d endRange()
    {
        return new Translation2d(radius, new Rotation2d(
            Math.toRadians(vectorBtoC.getAngle().getDegrees() + (90 * Math.signum(cornerAngle())))
        ));
    }

    public Translation2d[] getPoints(){
        int place = 0;
        Translation2d[] points = new Translation2d[(int)smoothnes];
        double diffAngle = endRange().getAngle().getDegrees() - startRange().getAngle().getDegrees();
        for (double i = 0; i < diffAngle; i = i + (diffAngle / smoothnes)) {
            points[place] = startRange().rotateBy((new Rotation2d(Math.toRadians(i)))).plus(getCenterCircle());
            System.out.println(points[place].plus(getCenterCircle()));
            place++;
        }
        return points;
    }






    
}
