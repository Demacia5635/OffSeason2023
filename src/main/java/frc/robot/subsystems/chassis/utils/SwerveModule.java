package frc.robot.subsystems.chassis.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants.SwerveModuleConstants;

import static frc.robot.Constants.ChassisConstants.*;
import static frc.robot.Constants.ChassisConstants.SwerveModuleConstants.*;

public class SwerveModule implements Sendable {
    private final TalonFX moveMotor;
    private final TalonFX angleMotor;
    private final CANCoder absoluteEncoder;
    private final double maxAngularVelocityChange = ANGULAR_ACCELERATION*Constants.CYCLE_DT;
    private final double maxDriveVelocityChange = DRIVE_ACCELERATION * Constants.CYCLE_DT;
    private Rotation2d optimizedAngle = new Rotation2d(0);

    private final double angleOffset;

    public SwerveModule(SwerveModuleConstants constants) {
        moveMotor = new TalonFX(constants.moveMotorId);
        angleMotor = new TalonFX(constants.angleMotorId);
        absoluteEncoder = new CANCoder(constants.absoluteEncoderId);

        angleOffset = constants.steerOffset;
        
        moveMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setNeutralMode(NeutralMode.Brake);

        angleMotor.configMotionCruiseVelocity(angularToEncoderSpeed(MAX_ANGULAR_VELOCITY));
        angleMotor.configMotionAcceleration(angularToEncoderSpeed(ANGULAR_ACCELERATION));
        angleMotor.configMotionSCurveStrength(1);

        setMovePID(MOVE_KP, 0 /*MOVE_KI*/, MOVE_KD);
        setAnglePIDF(ANGLE_VELOCITY_KP, 0 /*ANGLE_VELOCITY_KI*/, 
            ANGLE_VELOCITY_KD, ANGLE_KV);
    }

    public void setMovePID(double kP, double kI, double kD) {
        moveMotor.config_kP(0, kP);
        moveMotor.config_kI(0, kI);
        moveMotor.config_kD(0, kD);
    }

    public void setAnglePIDF(double kP, double kI, double kD, double kF) {
        angleMotor.config_kP(0, kP);
        angleMotor.config_kI(0, kI);
        angleMotor.config_kD(0, kD);
        angleMotor.config_kF(0, kF);
    }

    public void setInverted(boolean invert) {
        moveMotor.setInverted(invert);
    }

    /**
     * Returns the velocity of the module drive motor
     * @return Velocity in m/s
     */
    public double getVelocity() {
        return encoderToMetricSpeed(moveMotor.getSelectedSensorVelocity());
    }

    /**
     * Stops the module completely
     */
    public void stop() {
        setPower(0);
        setAngularPower(0);
    }

    /**
     * Sets the neutral mode of both motors
     */
    public void setNeutralMode(NeutralMode mode) {
        moveMotor.setNeutralMode(mode);
        angleMotor.setNeutralMode(mode);
    }

    /**
     * Sets the velocity of the module
     * @param v Velocity in m/s
     */
    public void setVelocity(double v) {
        double curVelocity = getVelocity();
        double dv = v - curVelocity;
        if(dv > maxDriveVelocityChange) {
            curVelocity += maxDriveVelocityChange;
        } else if(dv < -maxDriveVelocityChange) {
            curVelocity -= maxDriveVelocityChange;
        } else {
            curVelocity = v;
        }
        double volts = Math.signum(curVelocity)*MOVE_KS + MOVE_KV * curVelocity;
        
        moveMotor.set(ControlMode.Velocity, metricToEncoderSpeed(curVelocity), 
            DemandType.ArbitraryFeedForward, volts);
    }

    /**
     * Sets the power of the module
     * @param p Power in precent of the module (0%..100%)
     */
    public void setPower(double p) {
        moveMotor.set(ControlMode.PercentOutput, p);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition() - angleOffset);
    }

    /**
     * Sets the angle of the module with MotionMagic control
     */
    public void setAngle(Rotation2d angle) {
        angleMotor.set(ControlMode.MotionMagic, calculateTarget(angle.getDegrees()));
    }

    /**
     * Sets the rotational power of the module
     * @param p Power in precent of the module (0%..100%)
     */
    public void setAngularPower(double p) {
        angleMotor.set(ControlMode.PercentOutput, p);
    }

    /**
     * Returns the rotational velocity of the module
     * @return Velocity in deg/s
     */
    public double getAngularVelocity() {
        return encoderToAngularSpeed(angleMotor.getSelectedSensorVelocity());
    }

    /**
     * Sets the angular velocity of the module
     * @param v Velocity in deg/s
     */
    public void setAngularVelocity(double v) {
        double newVelocity = getAngularVelocity();
        if (Math.abs(newVelocity) < MAX_ANGULAR_VELOCITY)
            newVelocity += Math.signum(v) * ANGULAR_ACCELERATION * Constants.CYCLE_DT;
        double volts = ANGLE_KS + ANGLE_KV * v;
        if (Math.abs(v) >= ANGLE_KS) {
            angleMotor.set(ControlMode.Velocity, angularToEncoderSpeed(newVelocity), DemandType.ArbitraryFeedForward, volts / 12);
        }
        else
            angleMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Returns the state of the module
     * @return Velocity in m/s
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * Sets the state of the module
     */
    public void setState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        setVelocity(optimized.speedMetersPerSecond);
        optimizedAngle = optimized.angle; 
        setAngle(optimized.angle);
    }


    /**
     * Returns the module position
     * @return Position relative to the field
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(moveMotor.getSelectedSensorPosition() / PULSES_PER_METER, getAngle());
    }

    private double getAngleDifference(double target) {
        double difference = (target - getAngle().getDegrees()) % 360;
        return difference - ((int)difference / 180) * 360;
    }

    private double calculateTarget(double targetAngle) {
        double difference = getAngleDifference(targetAngle);
        return angleMotor.getSelectedSensorPosition() + (difference * PULSES_PER_DEGREE);
    }

    public static double metricToEncoderSpeed(double speed) {
        return speed * PULSES_PER_METER / 10;
    }

    public static double encoderToMetricSpeed(double speed) {
        return speed / PULSES_PER_METER * 10;
    }

    public static double angularToEncoderSpeed(double speed) {
        return speed * PULSES_PER_DEGREE / 10;
    }
    
    public static double encoderToAngularSpeed(double speed) {
        return speed / PULSES_PER_DEGREE * 10;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("raw angle", absoluteEncoder::getAbsolutePosition, null);
        builder.addDoubleProperty("angle", () -> getAngle().getDegrees(), null);
        builder.addDoubleProperty("velocity", this::getVelocity, null);
        builder.addDoubleProperty("angular velocity", this::getAngularVelocity, null);
        builder.addDoubleProperty("Wanted angle", () -> optimizedAngle.getDegrees(), null);
        builder.addDoubleProperty("Error", () ->  optimizedAngle.minus(getAngle()).getDegrees(), null);
    }
}