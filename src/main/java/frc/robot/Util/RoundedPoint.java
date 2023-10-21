
package frc.robot.Util;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

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

    /**
     * @return The angle of the corner in degrees, relative to the vector BtoA (Which represents one of the corner's wall)
     */
    private double cornerAngle(){
        double bigAngle = vectorAtoB.times(-1).getAngle().getDegrees();
        double smallAngle = vectorBtoC.getAngle().getDegrees();
        return smallAngle - bigAngle;
    }

    /**
     * 
     * @return A unit vector which represents the direction of the corner
     */
    private Translation2d getAngleVector(){
        Translation2d vector = new Translation2d(1, new Rotation2d(Math.toRadians(direction)));
        return vector;
    }

    /**
     * 
     * @return The length of the corner's angle cross, from point B to an encounter with the corner's circle
     */
    private double getLength(){
        double length = radius / Math.sin(Math.toRadians(Math.abs(cornerAngle() / 2)));
        System.out.println(length);
        return length;
    }

    /**
     * 
     * @return The position of the corner's circle
     */
    public Translation2d getCenterCircle(){
        Translation2d vector = getAngleVector().times(getLength());
        return vector.plus(bPoint);
    }

    /**
     * 
     * @return The starting position of the corner's curve (relative to the corner's circle's center)
     */
    public Translation2d startRange()
    {
        return new Translation2d(radius, new Rotation2d(
            Math.toRadians(vectorAtoB.times(-1).getAngle().getDegrees() - (90 *Math.signum(cornerAngle())))
        ));
    }


    /**
     * 
     * @return The ending position of the corner's curve (relative to the corner's circle's center)
     */
    public Translation2d endRange()
    {
        return new Translation2d(radius, new Rotation2d(
            Math.toRadians(vectorBtoC.getAngle().getDegrees() + (90 * Math.signum(cornerAngle())))
        ));
    }

    /**
     * 
     * @param pos
     * @param velocity
     * @return Returns a vector that represents the required velocity, According to the 
     */
    public Translation2d getCurrentVel(Translation2d pos,double velocity)
    {
        
        Translation2d relativePos = pos.minus(getCenterCircle());
        System.out.println("RelativPos : " + relativePos);
        System.out.println("StartRange : " + startRange());
        double diffAngle = endRange().getAngle().getDegrees() - startRange().getAngle().getDegrees();
        Translation2d unitVel = relativePos.rotateBy(new Rotation2d(Math.toRadians(90 * Math.signum(diffAngle)))).div(relativePos.getNorm());
        return unitVel.times(velocity * Constants.cycleTime);
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
