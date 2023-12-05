package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GripperConstants.*;

public class Gripper extends SubsystemBase {
    public TalonSRX motor = new TalonSRX(motorID);

    public Gripper() {
        SmartDashboard.putData(this);
    }

    private boolean isClosed() {
        return motor.isRevLimitSwitchClosed() == 1;
    }

    private boolean isOpened() {
        return motor.isFwdLimitSwitchClosed() == 1;
    }

    public void setPower(double pow) {
        motor.set(ControlMode.PercentOutput, pow);
    }

    public void stop() {
        setPower(0);
    }

    public GripperState getState() {
        if (isClosed() && !isOpened()){
        return GripperState.CLOSE;
        } else if(isOpened() && !isClosed()){
        return GripperState.OPEN;
        } else {
        return GripperState.BETWEEN;
        }
    }

    public enum GripperState{
        CLOSE,
        OPEN,
        BETWEEN;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addStringProperty("Gripper state", () -> (getState().toString()), null);
    }
}