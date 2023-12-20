package frc.robot.subsystems.chassis.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

public class SwerveModule implements Sendable {
    private final TalonFX moveMotor;
    private final TalonFX angleMotor;
    private final CANCoder absoluteEncoder;

    // private final SimpleMotorFeedforward moveFF;
    // private final SimpleMotorFeedforward angleFF;

    private double angleOffset;

    public SwerveModule(SwerveModuleConstants constants) {
        moveMotor = new TalonFX(constants.moveMotorId);
        angleMotor = new TalonFX(constants.angleMotorId);
        absoluteEncoder = new CANCoder(constants.absoluteEncoderId);
        angleOffset = constants.steerOffset;

        // moveFF = new SimpleMotorFeedforward(constants.kS, constants.kV, constants.kA);
        // angleFF = new SimpleMotorFeedforward(constants.kS, constants.kV, constants.kA);
    }

        angleMotor.configMotionCruiseVelocity(angularToEncoderSpeed(MAX_ANGULAR_VELOCITY));
        angleMotor.configMotionAcceleration(angularToEncoderSpeed(ANGULAR_ACCELERATION));
        angleMotor.configMotionSCurveStrength(1);

        setMovePID(MOVE_KP, MOVE_KI, MOVE_KD);
        setAnglePIDF(ANGLE_VELOCITY_KP, ANGLE_VELOCITY_KI, ANGLE_VELOCITY_KD, ANGLE_KV);
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

    public double getVelocity() {
        return moveMotor.getSelectedSensorVelocity() / PULSES_PER_METER * 10;
    }

    public void stop() {
        setPower(0, 0);
    }

    public void setVelocity(double v) {
        double newVelocity = getVelocity();
        if (Math.abs(newVelocity) < MAX_DRIVE_VELOCITY)
            newVelocity += Math.signum(v) * DRIVE_ACCELERATION * Constants.CYCLE_DT;
        double volts = MOVE_KS + MOVE_KV * v;
        if (Math.abs(v) > MOVE_KS)
            moveMotor.set(ControlMode.Velocity, metricToEncoderSpeed(newVelocity), DemandType.ArbitraryFeedForward, volts);
        else
            setPower(0);
    }

    public void setPower(double m, double s) {
        moveMotor.set(ControlMode.PercentOutput, m);
        angleMotor.set(ControlMode.PercentOutput, s);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition() - angleOffset);
    }

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

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState s = SwerveModuleState.optimize(state, getAngle());
        setVelocity(s.speedMetersPerSecond);
        setAngle(s.angle);
    }

    public void setInverted(boolean invert) {
        moveMotor.setInverted(invert);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(moveMotor.getSelectedSensorPosition() / PULSES_PER_METER, getAngle());
    }

    private double getAngleDifference(double current, double target) {
        double difference = (target - current) % 360;
        return difference - ((int)difference / 180) * 360;
    }

    private double calculateTarget(double targetAngle) {
        double difference = getAngleDifference(getAngle().getDegrees(), targetAngle);
        return angleMotor.getSelectedSensorPosition() + (difference * PULSES_PER_DEGREE);
    }
}
