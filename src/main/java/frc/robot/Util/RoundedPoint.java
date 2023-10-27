
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
     * @return The length of the corner's angle cross, from point B to the corner's circle's center
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
     * @return Returns a vector that represents the required velocity, According to the chassis position 
     */
    public Translation2d getVelDirection(Translation2d pos,double velocity)
    {
        
        Translation2d relativePos = pos.minus(getCenterCircle());
        System.out.println("RelativPos : " + relativePos);
        System.out.println("StartRange : " + startRange());
        System.out.println("EndRange : " + endRange());

        double diffAngle =  endRange().getAngle().getDegrees() - startRange().getAngle().getDegrees();
        System.out.println("diffAngle start to end : " + diffAngle);

        double psDiffAngle = relativePos.getAngle().getDegrees() - startRange().getAngle().getDegrees();
        double peDiffAngle =  relativePos.getAngle().getDegrees() - endRange().getAngle().getDegrees();
        System.out.println("diffAngle pos to start : " + psDiffAngle);
        System.out.println("diffAngle pos to end : " + peDiffAngle);
        System.out.println("sum : " + (psDiffAngle + peDiffAngle));

        Translation2d unitVel;
        if(peDiffAngle + psDiffAngle == 0)
        {
            System.out.println("curve");
            unitVel = relativePos.rotateBy(new Rotation2d(Math.toRadians(90 * Math.signum(diffAngle)))).div(relativePos.getNorm());
            
        }
        else
        {
            unitVel = new Translation2d(0,0);
        }
        
        
        
        return unitVel.times(velocity * Constants.cycleTime);
    }

    /**
     * 
     * @return An array of points that represent the corner's curve's structure
     */

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

    /**
     * 
     * @return the length of the corner's curve
     */
    public double getCurveLength()
    {
        double diffAngle = Math.abs(endRange().getAngle().getRadians() - startRange().getAngle().getRadians());
        return this.radius * diffAngle;
    }

    /**
     * 
     * @return The distance between point A and the start of the corner's curve
     */
    public double getAtoCurvelength()
    {
        return getCenterCircle().plus(startRange()).minus(aPoint).getNorm();
    }


    /**
     * 
     * @return The distance between point C and the end of the corner's curve
     */
    public double getCtoCurvelength()
    {
        return getCenterCircle().plus(endRange()).minus(cPoint).getNorm();
    }

    public double getTotalLength()
    {
        return getCtoCurvelength() + getCurveLength() + getAtoCurvelength();
    }

    
}
