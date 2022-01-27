// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaunchSubsystem extends SubsystemBase {

  private PWMVictorSPX launchTopMotor = new PWMVictorSPX(0);
  private PWMVictorSPX launchBottomMotor = new PWMVictorSPX(1);
  private PWMVictorSPX pushMotor = new PWMVictorSPX(2);
  private PWMVictorSPX tiltXMotor = new PWMVictorSPX(3);
  private PWMVictorSPX tiltYMotor = new PWMVictorSPX(4);
  private DigitalInput ballCheckSwitch = new DigitalInput(0);
  private MotorControllerGroup launchMotors = new MotorControllerGroup(launchTopMotor, launchBottomMotor);

  private float pushSpeed = 0.25f;
  private float intakeSpeed = 0.25f;

  public LaunchSubsystem() {
    launchBottomMotor.setInverted(true);
  }

  private float getVelocity(){
    //Put big boy equasion here
  }

  private float getMotorInput(){
    //Convert getVelocity() to motor inputs
  }

  public void shoot(){
    launchMotors.set(getMotorInput());
  }

  public void pushBall(){
    pushMotor.set(pushSpeed);
  }

  public void returnPush(){
    pushMotor.set(pushSpeed);
  }

  public void intakeBall(){
    pushMotor.set(intakeSpeed);
  }
}
