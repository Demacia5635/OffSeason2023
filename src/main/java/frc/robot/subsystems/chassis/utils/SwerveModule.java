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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ChassisConstants.SwerveModuleConstants;
import frc.robot.utils.Trapezoid;

import static frc.robot.Constants.ChassisConstants.*;
import static frc.robot.Constants.ChassisConstants.SwerveModuleConstants.*;

public class SwerveModule implements Sendable {
    private final TalonFX moveMotor;
    private final TalonFX angleMotor;
    private final CANCoder absoluteEncoder;

    private SimpleMotorFeedforward velocityFF;
    private SimpleMotorFeedforward angularFF;
    private Trapezoid angleTrapezoid;

    double targetVelocity = 0;
    double targetAngle = 0;

    private final double angleOffset;

    public boolean debug = false;

    public SwerveModule(SwerveModuleConstants constants) {
        moveMotor = new TalonFX(constants.moveMotorId);
        angleMotor = new TalonFX(constants.angleMotorId);
        absoluteEncoder = new CANCoder(constants.absoluteEncoderId);
        velocityFF = new SimpleMotorFeedforward(MOVE_KS, MOVE_KV);
        angularFF = new SimpleMotorFeedforward(ANGLE_KS, ANGLE_KV);
        angleTrapezoid = new Trapezoid(MAX_ANGULAR_VELOCITY, ANGULAR_ACCELERATION);

        moveMotor.configFactoryDefault();
        angleMotor.configFactoryDefault();


        angleOffset = constants.steerOffset;
        
        moveMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setNeutralMode(NeutralMode.Brake);

        angleMotor.configMotionCruiseVelocity(angularToEncoderSpeed(MAX_ANGULAR_VELOCITY));
        angleMotor.configMotionAcceleration(angularToEncoderSpeed(ANGULAR_ACCELERATION));
        angleMotor.configMotionSCurveStrength(1);

        setMovePID(MOVE_KP, MOVE_KI, MOVE_KD);
        setAnglePID(ANGLE_VELOCITY_KP, ANGLE_VELOCITY_KI, ANGLE_VELOCITY_KD);
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

    double lastMoveA = 0;
    double lastMoveV = 0;

    public void setVelocity(double v) {
        if(Math.abs(v) < 0.03) {
            setPower(0);
            return;
        }
        targetVelocity = v;
        double currentVelocity = getVelocity();
        if(lastMoveA > 0 && currentVelocity < lastMoveV && currentVelocity < v) {
            currentVelocity = lastMoveV;
        } else if(lastMoveA < 0 && currentVelocity > lastMoveV && currentVelocity < v) {
            currentVelocity = lastMoveV;
        }
        double tgtV = v;
        double maxAccel = DRIVE_ACCELERATION * Constants.CYCLE_DT;
        if (v > currentVelocity + maxAccel) {
            tgtV = currentVelocity + maxAccel;
        } else if (v < currentVelocity-maxAccel) {
            tgtV = currentVelocity - maxAccel;
        }
        double ff = velocityFF.calculate(tgtV);
        moveMotor.set(ControlMode.Velocity, metricToEncoderSpeed(tgtV), DemandType.ArbitraryFeedForward, ff);
        lastMoveA = tgtV - currentVelocity;
        lastMoveV = tgtV;
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
        targetAngle = angle.getDegrees();
    }

    public void update() {
        double dist = Rotation2d.fromDegrees(targetAngle).minus(getAngle()).getDegrees();
        double v = angleTrapezoid.calculate(dist, getAngularVelocity(), 0);
        if (Math.abs(dist) > MAX_STEER_ERROR)
            setAngularVelocity(v);
        else
            setAngularPower(0);
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

    public static double arbitraryFeedForwardAngle(double v) {
        return Math.signum(v)*ANGLE_KS + v*ANGLE_KV;
    }


    public static double arbitraryFeedForward(double v) {
        return Math.signum(v)*MOVE_KS + v*MOVE_KV;
    }


    /**
     * Sets the angular velocity of the module
     * @param v Velocity in deg/s
     */
    public void setAngularVelocityWithAccel(double v) {
        double currentVelocity = getAngularVelocity();
        double tgtV = v;
        double dv = v - currentVelocity;
        double maxAccel = ANGULAR_ACCELERATION * Constants.CYCLE_DT;
        if (dv > maxAccel && v > 0) {
            tgtV = currentVelocity + maxAccel;
        } else if (dv < -maxAccel && v < 0) {
            tgtV = currentVelocity - maxAccel;
        }
        double ff = angularFF.calculate(tgtV);

        angleMotor.set(ControlMode.Velocity, angularToEncoderSpeed(tgtV), DemandType.ArbitraryFeedForward, ff);
    }


    public void setAngularVelocity(double v) {
        double ff = angularFF.calculate(v);
        angleMotor.set(ControlMode.Velocity, angularToEncoderSpeed(v), DemandType.ArbitraryFeedForward, ff);
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
        builder.addDoubleProperty("desired angle", () -> targetAngle, null);
    }

}