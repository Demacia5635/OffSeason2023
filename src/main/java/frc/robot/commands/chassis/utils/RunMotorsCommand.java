package frc.robot.commands.chassis.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;

public class RunMotorsCommand extends CommandBase {
    private final double power;
    private final double pulsesPerUnit;
    private final TalonFX[] motors;

    public RunMotorsCommand(Chassis chassis, double pulsesPerUnit, double power, int[] ids) {
        this.power = power;
        this.pulsesPerUnit = pulsesPerUnit;

        motors = new TalonFX[ids.length];
        for (int i = 0; i < ids.length; i++) {
            motors[i] = new TalonFX(ids[i]);
        }

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        for (TalonFX motor : motors) {
            motor.set(ControlMode.PercentOutput, power);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("velocity: " + motors[0].getSelectedSensorVelocity() * pulsesPerUnit / 10);
        for (TalonFX motor : motors) {
            motor.set(ControlMode.PercentOutput, 0);
        }
    }
}