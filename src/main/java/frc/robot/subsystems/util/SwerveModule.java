
package frc.robot.subsystems.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.simulation.Motor;
import static frc.robot.Constants.*;

public class SwerveModule {
    private Motor angleMotor;
    private Motor moveMotor;
    public SwerveModule(){
        angleMotor = new Motor();
        moveMotor = new Motor();
    }
    public void stop(){
        angleMotor.setVelocity(0);
        moveMotor.setVelocity(0);
    }
    public void setVelocity(double v) {
        moveMotor.setVelocity(v);
    }
    public double getVelocity() {
        return moveMotor.getVelocity();
    } 
    public double getDistance(){
        return moveMotor.getDistance();
    }
    public void setAngle(Rotation2d angle) {
        angleMotor.setPosition(calculateTarget(angle.getDegrees()));
    }
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angleMotor.getDistance());
    }
    public void setState(SwerveModuleState state) {
        setVelocity(state.speedMetersPerSecond);
        setAngle(state.angle);
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }
    public void update() {
        moveMotor.update();
        angleMotor.update();
    }
    public static double getAngleDifference(double current, double target) {
        double difference = (target - current) % 360;
        return difference - ((int)difference / 180) * 360;
    }

    private double calculateTarget(double targetAngle) {
        double difference = getAngleDifference(getAngle().getDegrees(), targetAngle);
        return angleMotor.getDistance() + (difference * pulsesPerDegree);
    }
}
