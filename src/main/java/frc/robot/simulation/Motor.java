package frc.robot.simulation;
import static frc.robot.Constants.*;

public class Motor {
    private double velocity;
    private double pulses;


    /**
     *  update the motor velocity
     * @param V velocity in m/s
     */

    public void setVelocity(double V){
        velocity = V;
    }
    public double getVelocity(){
        return velocity;
    }
    public void setPosition(double pulses){
        this.pulses = pulses;
    }

    public void update(){
        pulses += velocity * countPerMeter * cycleTime;
    }


    /**
     *  update the motor distance
     * @return in pulses
     */
    public double getDistance(){
        return pulses;
    }


}
