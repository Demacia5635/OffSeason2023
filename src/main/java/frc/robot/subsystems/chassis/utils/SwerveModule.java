package frc.robot.subsystems.chassis.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants.SwerveModuleConstants;

import static frc.robot.Constants.ChassisConstants.*;

public class SwerveModule {
    private final TalonFX moveMotor;
    private final TalonFX angleMotor;
    private final CANCoder absoluteEncoder;

    private final SimpleMotorFeedforward moveFF;
    private final SimpleMotorFeedforward angleFF;

    private double angleOffset;

    public SwerveModule(SwerveModuleConstants constants) {
        moveMotor = new TalonFX(constants.moveMotorId);
        angleMotor = new TalonFX(constants.angleMotorId);
        absoluteEncoder = new CANCoder(constants.absoluteEncoderId);

        moveFF = new SimpleMotorFeedforward(constants.kS, constants.kV, constants.kA);
        angleFF = new SimpleMotorFeedforward(constants.kS, constants.kV, constants.kA);
    }

    public void calibrateOffset() {
        angleOffset = absoluteEncoder.getAbsolutePosition();
    }

    public void setMovePID(double kP, double kI, double kD) {
        moveMotor.config_kP(0, kP);
        moveMotor.config_kP(0, kI);
        moveMotor.config_kP(0, kD);
    }

    public void setAnglePID(double kP, double kI, double kD) {
        angleMotor.config_kP(0, kP);
        angleMotor.config_kP(0, kI);
        angleMotor.config_kP(0, kD);
    }

    public double getVelocity() {
        return moveMotor.getSelectedSensorVelocity() * PULSES_PER_METER * 10;
    }

    public void stop() {
        setPower(0);
    }

    public void setVelocity(double v) {
        double volts = moveFF.calculate(getVelocity(), v, Constants.CYCLE_DT);
        moveMotor.set(ControlMode.Velocity, v * PULSES_PER_METER / 10, DemandType.ArbitraryFeedForward, volts / 12);
    }

    public void setPower(double power) {
        moveMotor.set(ControlMode.PercentOutput, power);
        angleMotor.set(ControlMode.PercentOutput, power);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition() - angleOffset);
    }

    public void setAngle(Rotation2d angle) {
        double volts = angleFF.calculate(ANGULAR_VELOCITY, ANGULAR_ACCELERATION);
        moveMotor.set(ControlMode.MotionMagic, calculateTarget(angle.getDegrees()), DemandType.ArbitraryFeedForward, volts);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public void setState(SwerveModuleState state) {
        setVelocity(state.speedMetersPerSecond);
        setAngle(state.angle);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(moveMotor.getSelectedSensorPosition() * PULSES_PER_METER, getAngle());
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
