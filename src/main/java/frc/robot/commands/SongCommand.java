package frc.robot.commands;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SongCommand extends CommandBase {
  TalonFX m;
  private final Orchestra orchestra;

  public SongCommand() {
    m = new TalonFX(7);
    ArrayList<TalonFX> arr = new ArrayList<>();
    arr.add(new TalonFX(30));
    orchestra = new Orchestra(arr, Filesystem.getDeployDirectory().getPath() + "/song.chrp");
  }

  @Override
  public void initialize() {
      orchestra.play();
  }
}
