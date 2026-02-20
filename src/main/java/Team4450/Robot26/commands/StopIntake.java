package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class StopIntake extends Command {
  private Intake intake;

  public StopIntake(Intake intake) {
    this.intake = intake;
  }

  public void initialize() {

  }

  public void execute() {
    intake.stopIntake();
  }

  public boolean isFinished() {
    return true;

  }

  public void end() {

  }
}
