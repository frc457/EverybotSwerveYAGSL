package frc.robot.commands;

import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.RollerSubsystem;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj2.command.Command;

/** A command to remove (score or pass) Algae. */
public class AutoAlgaeOutCommand extends Command {
  private final RollerSubsystem m_roller;
  private boolean finished = false;

 

  /**
   * Rolls the Algae out of the intake. 
   * We recommend not using this to score coral.
   *
   * @param roller The subsystem used by this command.
   */
  public AutoAlgaeOutCommand(RollerSubsystem roller) {
    m_roller = roller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 finished = false;

    TimerTask task = new TimerTask() {
      public void run() {
        finished = true;
      }
    };

    Timer timer = new Timer();

    timer.schedule(task, 1000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_roller.runRoller(RollerConstants.Auto_ROLLER_ALGAE_OUT);
  }

  // Called once the command ends or is interrupted. This ensures the roller is not running when not intented.
  @Override
  public void end(boolean interrupted) {
    m_roller.runRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}

