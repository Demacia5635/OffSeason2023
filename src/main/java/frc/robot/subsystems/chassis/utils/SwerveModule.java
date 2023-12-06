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
import frc.robot.Constants.ChassisConstants.SwerveModuleConstants;
import frc.robot.utils.Trapezoid;

import static frc.robot.Constants.ChassisConstants.*;
import static frc.robot.Constants.ChassisConstants.SwerveModuleConstants.*;

public class SwerveModule implements Sendable {
    private final TalonFX moveMotor;
    private final TalonFX angleMotor;
    private final CANCoder absoluteEncoder;

    private final double angleOffset;
    
    private Rotation2d desiredAngle;

    public SwerveModule(SwerveModuleConstants constants) {
        moveMotor = new TalonFX(constants.moveMotorId);
        angleMotor = new TalonFX(constants.angleMotorId);
        absoluteEncoder = new CANCoder(constants.absoluteEncoderId);

        angleOffset = constants.steerOffset;
        
        moveMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setNeutralMode(NeutralMode.Brake);

        desiredAngle = new Rotation2d();

        angleMotor.configMotionCruiseVelocity(ANGULAR_VELOCITY * PULSES_PER_DEGREE / 10);
        angleMotor.configMotionAcceleration(ANGULAR_ACCELERATION * PULSES_PER_DEGREE / 10);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("raw angle", absoluteEncoder::getAbsolutePosition, null);
        builder.addDoubleProperty("angle", () -> getAngle().getDegrees(), null);
        builder.addDoubleProperty("velocity", this::getVelocity, null);
        builder.addDoubleProperty("angular velocity", this::getAngularVelocity, null);
    }

    public void setMovePID(double Kp, double Ki, double Kd) {
        moveMotor.config_kP(0, Kp);
        moveMotor.config_kI(0, Ki);
        moveMotor.config_kD(0, Kd);
    }

    public void setAnglePID(double Kp, double Ki, double Kd) {
        angleMotor.config_kP(0, Kp);
        angleMotor.config_kI(0, Ki);
        angleMotor.config_kD(0, Kd);
    }

    public double getVelocity() {
        return moveMotor.getSelectedSensorVelocity() / PULSES_PER_METER * 10;
    }

    public void stop() {
        setPower(0, 0);
    }

    public void setVelocity(double v) {
        double volts = MOVE_KS + MOVE_KV * v;
        if (Math.abs(v) > 0.1)
            moveMotor.set(ControlMode.Velocity, metricToEncoderSpeed(v), DemandType.ArbitraryFeedForward, volts * 0.02);
        else
            moveMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setNeutralMode(NeutralMode mode) {
        moveMotor.setNeutralMode(mode);
        angleMotor.setNeutralMode(mode);
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

    public double getAngularVelocity() {
        return angleMotor.getSelectedSensorVelocity() / PULSES_PER_DEGREE * 10;
    }

    public void setAngularVelocity(double v) {
        double volts = ANGLE_KS + ANGLE_KV * v;
        if (Math.abs(v) > 10) {
            angleMotor.set(ControlMode.Velocity, v * PULSES_PER_DEGREE / 10, DemandType.ArbitraryFeedForward, volts * 0.02);
        }
        else
            angleMotor.set(ControlMode.PercentOutput, 0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        setVelocity(optimized.speedMetersPerSecond);
        setAngle(optimized.angle);
    }

    public void setInverted(boolean invert) {
        moveMotor.setInverted(invert);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(moveMotor.getSelectedSensorPosition() * PULSES_PER_METER, getAngle());
    }

    public double getAngleDifference(double target) {
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
}
