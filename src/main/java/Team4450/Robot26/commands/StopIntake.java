package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class StopIntake extends Command {
  private Intake intake;

  private State state;

  private static enum State {
    STOPPING, INTAKE, END
  };

  public StopIntake(Intake intake) {
    this.intake = intake;
  }

  public void initialize() {
    state = State.STOPPING;

  }

  public void execute() {
    intake.stopIntake();
  }

  public boolean isFinished() {
    return state == State.END;

  }

  public void end() {

  }
}
