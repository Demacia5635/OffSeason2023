
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
    Rotation2d cornerDir;
    Rotation2d cornerAngle;

    public RoundedPoint(pathPoint aPoint, pathPoint bPoint, pathPoint cPoint){
        this.radius = bPoint.getRadius();
        this.aPoint = aPoint.getTranslation();
        this.bPoint = bPoint.getTranslation();
        this.cPoint = cPoint.getTranslation();
        
        vectorAtoB = this.bPoint.minus(this.aPoint);
        vectorBtoC = this.cPoint.minus(this.bPoint);



        this.cornerAngle = vectorAtoB.times(-1).getAngle().minus(vectorBtoC.getAngle());
        if(Math.abs(cornerAngle.getDegrees()) >= 175 && Math.abs(cornerAngle.getDegrees()) <= 180)
            this.radius = 0;
        else{
            if(radius > getMaxRadius()){
                radius = getMaxRadius();
                System.out.println("radius is bigger then possible, new radius is: " + radius);
            }
    
        }
        this.cornerDir = vectorAtoB.times(-1).getAngle().minus(this.cornerAngle.div(2));

        
        

    }
    
    
    public double getMaxRadius()
    {
        return Math.sin(Math.abs(this.cornerAngle.getRadians()) / 2) * Math.min(vectorAtoB.getNorm(), vectorBtoC.getNorm());
    }

    

    /**
     * 
     * @return The position of the corner's circle center
     */
    //Calculate Cross Angle vector here too
    public Translation2d getCenterCircle(){
        double length;

        if(this.cornerAngle.div(2).getSin() != 0 && Math.abs(this.cornerAngle.getDegrees()) < 177)
            length =  radius / Math.abs(this.cornerAngle.div(2).getSin());
        else
            length = 0;
        
        Translation2d dirVector = new Translation2d(length, this.cornerDir);
        return dirVector.plus(bPoint);
    }

    /**
     * 
     * @return The starting position of the corner's curve (relative to the corner's circle's center)
     */
    //Calculate cornerDegree beforehand.
    public Translation2d startRange()
    {
        return new Translation2d(radius, vectorAtoB.times(-1).getAngle().plus(new Rotation2d((Math.PI/2) * Math.signum(cornerAngle.getDegrees()))));
    }
    /**
     * 
     * @return The ending position of the corner's curve (relative to the corner's circle's center) 
     */
    public Translation2d endRange()
    {
        return new Translation2d(radius, vectorBtoC.getAngle().minus(new Rotation2d((Math.PI/2) * Math.signum(cornerAngle.getDegrees()))));
    }

    // /**
    //  * 
    //  * @param position chassis position in Translation2d
    //  * @param velocity chassis velocity in m/s
    //  * @return Returns a vector that represents the required velocity, According to the chassis position 
    //  */
    // public Translation2d getVelDirection(Translation2d pos,double velocity)
    // {

    //     //calculates the position relative to the center circle
    //     Translation2d relativePos = pos.minus(getCenterCircle());
    //     System.out.println("RelativPos : " + relativePos);
    //     System.out.println("StartRange : " + startRange());
    //     System.out.println("EndRange : " + endRange());

    //     //calculates the angle between the start and the end of the circle
    //     double diffAngle =  endRange().getAngle().getDegrees() - startRange().getAngle().getDegrees();
    //     System.out.println("diffAngle start to end : " + diffAngle);

    //     //calculates the relative angle between the start and the end of the circle
    //     double startDiffAngle = relativePos.getAngle().getDegrees() - startRange().getAngle().getDegrees();
    //     double endDiffAngle =  relativePos.getAngle().getDegrees() - endRange().getAngle().getDegrees();
    //     System.out.println("diffAngle pos to start : " + startDiffAngle);
    //     System.out.println("diffAngle pos to end : " + endDiffAngle);
    //     System.out.println("sum : " + (startDiffAngle + endDiffAngle));

    //     Translation2d unitVel;
    //     if(endDiffAngle + startDiffAngle == 0)
    //     {
    //         System.out.println("curve");
    //         unitVel = relativePos.rotateBy(new Rotation2d(Math.toRadians(90 * Math.signum(diffAngle)))).div(relativePos.getNorm());
            
    //     }
    //     else
    //     {
    //         unitVel = new Translation2d(0,0);
    //     }
        
        
        
    //     return unitVel.times(velocity * Constants.cycleTime);
    // }

    /**
     * 
     * @return An array of points that represent the corner's curve's structure
     */

    /*public Translation2d[] getPoints(){
        int place = 0;
        Translation2d[] points = new Translation2d[(int)smoothnes];
        double diffAngle = endRange().getAngle().getDegrees() - startRange().getAngle().getDegrees();
        for (double i = 0; i < diffAngle; i = i + (diffAngle / smoothnes)) {
            points[place] = startRange().rotateBy((new Rotation2d(Math.toRadians(i)))).plus(getCenterCircle());
            System.out.println(points[place].plus(getCenterCircle()));
            place++;
        }
        return points;
    } */

    /**
     * 
     * @return the length of the corner's curve
     */
    public double getCurveLength()
    {
        double diffAngle = Math.abs(endRange().getAngle().minus(startRange().getAngle()).getRadians());
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

    public Translation2d getCurveStart()
    {
        return getCenterCircle().plus(startRange());
    }

    public Translation2d getCurveEnd()
    {
        return getCenterCircle().plus(endRange());
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

    // public ArcDrive getArcDrive(Chassis chassis)
    // {
    //     return new ArcDrive(chassis, getCenterCircle().plus(startRange()), getCenterCircle(), endRange().getAngle().minus(startRange().getAngle()), 5);
    // }

    public Leg getAtoCurveLeg()
    {
        return new Leg(aPoint, startRange().plus(getCenterCircle()), false);
    }
    public Leg getCtoCurveLeg()
    {
        return new Leg(endRange().plus(getCenterCircle()), cPoint, false);
    }

    public Arc getArc()
    {
        Rotation2d diffAngle = endRange().getAngle().minus(startRange().getAngle());
        return new Arc(startRange().plus(getCenterCircle()), getCenterCircle(), diffAngle, false);
    }
}
