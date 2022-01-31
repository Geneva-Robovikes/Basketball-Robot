// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LaunchSubsystem;

/** An example command that uses an example subsystem. */
public class LaunchBall extends CommandBase {
  private final LaunchSubsystem launchSubsystem;
  private final PushBall pushBall;
  private double speed = 0.9;
  private double waitTime = 1.0;
  private double waitForCheck = 0.5;
  private double requiredDistance;
  private Timer velCheckTimer;
  private Timer timer;
  private boolean firstTime;

  public LaunchBall(LaunchSubsystem subsystem, PushBall pushBall) {
    launchSubsystem = subsystem;
    this.pushBall = pushBall;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launchSubsystem.setLauncherSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(velCheckTimer.hasElapsed(waitForCheck)) {
      try {
        requiredDistance = launchSubsystem.RequiredDistance(launchSubsystem.getWheelVelocity());
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      velCheckTimer.stop();
      velCheckTimer.reset();
    }
    if(requiredDistance < launchSubsystem.getDistance() * 39.3700787 + 1 && requiredDistance > launchSubsystem.getDistance() * 39.3700787 - 1 && firstTime) {
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launchSubsystem.setLauncherSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.hasElapsed(waitTime)){
      CommandScheduler.getInstance().schedule(pushBall);
      return true;
    }
    else{return false;}
  }
}