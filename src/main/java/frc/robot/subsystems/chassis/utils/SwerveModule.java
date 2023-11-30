package frc.robot.subsystems.chassis.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants.SwerveModuleConstants;
import frc.robot.utils.Trapezoid;

import static frc.robot.Constants.ChassisConstants.*;
import static frc.robot.Constants.ChassisConstants.SwerveModuleConstants.*;

public class SwerveModule implements Sendable {
    private final double MAX_VELOCITY_CHANGE = ACCELERATION * Constants.CYCLE_DT;

    private final TalonFX moveMotor;
    private final TalonFX angleMotor;
    private final CANCoder absoluteEncoder;

    private final Trapezoid trapezoid;

    private final SimpleMotorFeedforward moveFF;
    private final SimpleMotorFeedforward angleFF;

    private double angleOffset;

    public SwerveModule(SwerveModuleConstants constants) {
        moveMotor = new TalonFX(constants.moveMotorId);
        angleMotor = new TalonFX(constants.angleMotorId);
        absoluteEncoder = new CANCoder(constants.absoluteEncoderId);
        angleOffset = constants.steerOffset;

        moveFF = new SimpleMotorFeedforward(MOVE_KS, MOVE_KV, MOVE_KA);
        angleFF = new SimpleMotorFeedforward(ANGLE_KS, ANGLE_KV, ANGLE_KA);
        
        moveMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setNeutralMode(NeutralMode.Brake);

        setMovePID(MOVE_KP, MOVE_KI, MOVE_KD);
        setAnglePID(ANGLE_KP, ANGLE_KI, ANGLE_KD);

        trapezoid = new Trapezoid(ANGULAR_VELOCITY, ANGULAR_ACCELERATION);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("offset", () -> absoluteEncoder.getAbsolutePosition(), null);
    }

    public void calibrateOffset() {
        angleOffset = absoluteEncoder.getAbsolutePosition();
    }

    public void setMovePID(double kP, double kI, double kD) {
        moveMotor.config_kP(0, kP);
        moveMotor.config_kI(0, kI);
        moveMotor.config_kD(0, kD);
    }

    public void setAnglePID(double kP, double kI, double kD) {
        angleMotor.config_kP(0, kP);
        angleMotor.config_kI(0, kI);
        angleMotor.config_kD(0, kD);
    }

    public double getVelocity() {
        return moveMotor.getSelectedSensorVelocity() / PULSES_PER_METER * 10;
    }

    public void stop() {
        setPower(0, 0);
    }

    public void setVelocity(double v) {
        double currentVelocity = getVelocity();
        double deltaVelocity = v - currentVelocity;
        if (Math.abs(deltaVelocity) > MAX_VELOCITY_CHANGE)
            v = currentVelocity +  Math.signum(deltaVelocity) * MAX_VELOCITY_CHANGE;
        double volts = moveFF.calculate(v, 0);
        moveMotor.set(ControlMode.Velocity, metricToEncoderSpeed(v));
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
        double velocity = trapezoid.calculate(getAngleDifference(getAngle().getDegrees(), angle.getDegrees()).getDegrees(), getAngularVelocity(), ANGULAR_VELOCITY);
        double volts = angleFF.calculate(ANGULAR_VELOCITY, ANGULAR_ACCELERATION);
        angleMotor.set(ControlMode.Position, calculateTarget(angle.getDegrees()));
    }

    public double getAngularVelocity() {
        return angleMotor.getSelectedSensorVelocity() / PULSES_PER_METER * 10;
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

    private Rotation2d getAngleDifference(double current, double target) {
        return Rotation2d.fromDegrees(target).minus(Rotation2d.fromDegrees(current));
    }

    private double calculateTarget(double targetAngle) {
        double difference = getAngleDifference(getAngle().getDegrees(), targetAngle).getDegrees();
        return angleMotor.getSelectedSensorPosition() + (difference * PULSES_PER_DEGREE);
    }

    public static double metricToEncoderSpeed(double speed) {
        return speed * PULSES_PER_METER / 10;
    }
}
