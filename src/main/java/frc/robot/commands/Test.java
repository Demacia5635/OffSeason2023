
package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.RoundedPoint;


public class Test extends CommandBase {
  double radius = 0.1;
  Translation2d aPoint = new Translation2d(-1, 0);
  Translation2d bPoint = new Translation2d(0, -1);
  Translation2d cPoint = new Translation2d(1, 0);
  RoundedPoint roundedPoint = new RoundedPoint(radius, aPoint, bPoint, cPoint);

  public Test() {


  }


  @Override
  public void initialize() {
    System.out.println(roundedPoint.getCenterCircle()); 
  }


  @Override
  public void execute() {

    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
