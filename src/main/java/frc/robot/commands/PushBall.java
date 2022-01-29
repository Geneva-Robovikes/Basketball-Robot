// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LaunchSubsystem;

/** An example command that uses an example subsystem. */
public class PushBall extends CommandBase {
  private final LaunchSubsystem launchSubsystem;
  private Timer timer;
  private double timerWait = 1;
  private boolean firstTime = true;
  private double pushSpeed = 0.25f;

  public PushBall(LaunchSubsystem subsystem) {
    launchSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launchSubsystem.setPushSpeed(pushSpeed);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.hasElapsed(timerWait) && firstTime) {
      launchSubsystem.setPushSpeed(-pushSpeed);
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launchSubsystem.setPushSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.hasElapsed(timerWait)){return true;}
    return false;
  }
}