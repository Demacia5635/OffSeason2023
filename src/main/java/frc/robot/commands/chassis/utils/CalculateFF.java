package frc.robot.commands.chassis.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;

public class CalculateFF extends CommandBase {

    double[] p = {0.2, -0.2, 0.3, -0.3, 0.4, -0.4};
    double cycleTime = 0;
    double cycle = 2;
    double[] v = new double[p.length];
    int c;
    Chassis chassis;

    public CalculateFF(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);

    } 

    @Override
    public void initialize() {
        chassis.resetWheels();
        c = 0;
        cycleTime = -1;
    }

    @Override
    public void execute() {
        if(cycleTime < 0) {
            chassis.setModulesPower(p[c]);
            cycleTime = Timer.getFPGATimestamp();
        }
        double time = Timer.getFPGATimestamp();
        if(time > cycleTime + cycle) {
            v[c] = chassis.getVelocity().getNorm();
            c++;
            if(c < p.length) {
                chassis.setModulesPower(p[c]);
                cycleTime = Timer.getFPGATimestamp();
            } else {
                chassis.setModulesPower(0);
            }
        }

    }

    @Override
    public boolean isFinished() {
        return c >= p.length;
    }

    @Override
    public void end(boolean interrupted) {
        for(int i = 0; i < p.length; i++)  {
            System.out.println("p = " + p[i] + " v=" + v[i]);
        }
    }
    
}
