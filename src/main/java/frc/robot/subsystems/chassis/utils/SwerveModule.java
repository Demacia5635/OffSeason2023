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

import static frc.robot.Constants.ChassisConstants.*;
import static frc.robot.Constants.ChassisConstants.SwerveModuleConstants.*;

import javax.swing.text.StyledEditorKit.FontFamilyAction;

public class SwerveModule implements Sendable {
    private final TalonFX moveMotor;
    private final TalonFX angleMotor;
    private final CANCoder absoluteEncoder;

    SimpleMotorFeedforward velocityFF;

    double tgtVelocity = 0;
    double tgtAngle = 0;

    private final double angleOffset;

    public SwerveModule(SwerveModuleConstants constants) {
        moveMotor = new TalonFX(constants.moveMotorId);
        angleMotor = new TalonFX(constants.angleMotorId);
        absoluteEncoder = new CANCoder(constants.absoluteEncoderId);
        velocityFF = new SimpleMotorFeedforward(ANGLE_KS, ANGLE_KV);

        angleOffset = constants.steerOffset;
        
        moveMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setNeutralMode(NeutralMode.Brake);

        angleMotor.configMotionCruiseVelocity(angularToEncoderSpeed(MAX_ANGULAR_VELOCITY));
        angleMotor.configMotionAcceleration(angularToEncoderSpeed(ANGULAR_ACCELERATION));
        angleMotor.configMotionSCurveStrength(1);

        setMovePID(MOVE_KP,0 /*MOVE_KI*/ , MOVE_KD);
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
        tgtVelocity = v;
        double curVelocity = getVelocity();
        double tgtV = v;
        double dv = v - curVelocity;
        double max = DRIVE_ACCELERATION * Constants.CYCLE_DT;
        if(dv > max && v > 0) {
            tgtV = curVelocity + max;
        } else if(dv < -max && v < 0) {
            tgtV = curVelocity - max;
        }
        double ff = velocityFF.calculate(tgtV);
        moveMotor.set(ControlMode.Velocity, metricToEncoderSpeed(tgtV), DemandType.ArbitraryFeedForward, ff);
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
        tgtAngle = angle.getDegrees();
        angleMotor.set(ControlMode.MotionMagic, calculateTarget(angle));
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

    public static double ArbitraryFeedForwardAngle(double v) {
        return ANGLE_KS + v*ANGLE_KV;
    }


    public static double ArbitraryFeedForward(double v) {
        return MOVE_KS + v*MOVE_KV;
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
        setAngle(optimized.angle);
    }

    /**
     * Returns the module position
     * @return Position relative to the field
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(moveMotor.getSelectedSensorPosition() / PULSES_PER_METER, getAngle());
    }

    private double getAngleDifference(Rotation2d target) {
        Rotation2d curAngle = getAngle();
        return target.minus(curAngle).getDegrees();
    }

    private double calculateTarget(Rotation2d targetAngle) {
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
        builder.addDoubleProperty("tgt velocity", ()->tgtVelocity, null);
        // builder.addDoubleProperty("tgt angel",()->tgtAngle, null);
        // builder.addDoubleProperty("vel error",()->moveMotor.getClosedLoopError(), null);
        // builder.addDoubleProperty("vel tgt",()->moveMotor.getClosedLoopTarget(), null);
        builder.addDoubleProperty("angular module velocity", this::getAngularVelocity, null);

    }

}
