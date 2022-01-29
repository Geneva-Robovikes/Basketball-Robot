// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LaunchSubsystem;

/** An example command that uses an example subsystem. */
public class GroundYTilt extends CommandBase {
  private final LaunchSubsystem launchSubsystem;
  private float tiltSpeed = -0.1f;

  public GroundYTilt(LaunchSubsystem subsystem) {
    launchSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launchSubsystem.setYTiltSpeed(tiltSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launchSubsystem.setYTiltSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Check if the back limit switch is pressed then return true
    if(launchSubsystem.tiltGroundSwitch.get()) {return true;}
    else {return false;}
  }
}