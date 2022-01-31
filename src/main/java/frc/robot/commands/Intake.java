// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LaunchSubsystem;

/** An example command that uses an example subsystem. */
public class Intake extends CommandBase {
  private final LaunchSubsystem launchSubsystem;
  private final RobotContainer robotContainer;
  private float intakeSpeed = -0.25f;

  public Intake(LaunchSubsystem subsystem, RobotContainer container) {
    robotContainer = container;
    launchSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launchSubsystem.setLauncherSpeed(intakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launchSubsystem.setLauncherSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Check if the back limit switch is pressed then return true
    if(launchSubsystem.ballCheckSwitch.get()) {
      CommandScheduler.getInstance().schedule(robotContainer.launchBall);
      return true;
    }
    else {return false;}
  }
}