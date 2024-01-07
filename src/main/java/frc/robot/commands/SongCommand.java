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
    m = new TalonFX(0); // TODO: set motor id
    ArrayList<TalonFX> arr = new ArrayList<>();
    arr.add(m);
    orchestra = new Orchestra(arr, Filesystem.getDeployDirectory().getPath() + "/song.chrp");
  }

  @Override
  public void initialize() {
      orchestra.play();
  }
}
