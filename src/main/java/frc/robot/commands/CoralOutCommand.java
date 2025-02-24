// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.RollerSubsystem;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj2.command.Command;

/** An CoralOutCommand that uses a roller subsystem. */
public class CoralOutCommand extends Command {
  private final RollerSubsystem m_roller;
  private boolean finished = false;

  /**
   * Use to score coral into L1.
   *
   * @param roller The subsystem used by this command.
   */
  public CoralOutCommand(RollerSubsystem roller) {
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
    m_roller.runRoller(RollerConstants.ROLLER_CORAL_OUT);
  }

  // Called once the command ends or is interrupted. Ensures the roller
  // is not running after we let go of the button. 
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
