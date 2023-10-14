package frc.robot.simulation;


public class Gyro {
    private double velocity;
    private double angle;


    /**
     *  update the gyro velocity
     * @param V velocity in degrees/s
     */

    public void setVelocity(double V){
        velocity = V;
    }

    public void update(){
        angle += velocity;
    }


    /**
     *  update the gyro angle
     * @return degrees
     */
    public double getAngle(){
        return angle;
    }


}
