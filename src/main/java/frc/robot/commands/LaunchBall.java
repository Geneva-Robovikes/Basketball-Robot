// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LaunchSubsystem;

/** An example command that uses an example subsystem. */
public class LaunchBall extends CommandBase {
  private final LaunchSubsystem launchSubsystem;
  private double velocity;
  private double currentVelocity;
  private boolean readyToFire;

  public LaunchBall(LaunchSubsystem subsystem) {
    launchSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    velocity = launchSubsystem.GetVelocity(launchSubsystem.getEncoderDistance() + 15, 45);
    System.out.println(velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      currentVelocity = launchSubsystem.getWheelVelocity();
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    if(currentVelocity - 0.5 < velocity) {
      if(currentVelocity + 0.5 > velocity){
        readyToFire = true;
      }
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
    if(readyToFire){
      return true;
    }
    return false;
  }
}