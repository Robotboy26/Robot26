package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class StartIntake extends Command {
  private Intake intake;

  private static enum State {
    MOVING, INTAKE, STOP
  };

  public StartIntake(Intake intake) {
    this.intake = intake;
  }

  public void initialize() {
    state = State.MOVING;

  }

  public void execute() {
    intake.startIntake();
  }

  public boolean isFinished() {
    return true;

  }

  public void end() {

  }
}
