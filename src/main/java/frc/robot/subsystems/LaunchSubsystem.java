// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaunchSubsystem extends SubsystemBase {

  private PWMVictorSPX launchTopMotor = new PWMVictorSPX(0);
  private PWMVictorSPX launchBottomMotor = new PWMVictorSPX(1);
  private PWMVictorSPX pushMotor = new PWMVictorSPX(2);
  private PWMVictorSPX tiltXMotor = new PWMVictorSPX(3);
  private PWMVictorSPX tiltYMotor = new PWMVictorSPX(4);
  private MotorControllerGroup launchMotors = new MotorControllerGroup(launchTopMotor, launchBottomMotor);
  public DigitalInput ballCheckSwitch = new DigitalInput(0);
  public DigitalInput tiltGroundSwitch = new DigitalInput(1);

  public LaunchSubsystem() {
    launchBottomMotor.setInverted(true);
  }

  public void setLauncherSpeed(double speed) {
    launchMotors.set(speed);
  }

  public double getLauncherSpeed(){
    return launchMotors.get();
  }

  public void setPushSpeed(double speed) {
    pushMotor.set(speed);
  }

  public double getPushSpeed(){
    return pushMotor.get();
  }

  public void setYTiltSpeed(double yTiltSpeed) {
    tiltYMotor.set(yTiltSpeed);
  }

  public double getYTiltSpeed(){
    return tiltYMotor.get();
  }

  public void setXTiltSpeed(double xTiltSpeed) {
    tiltXMotor.set(xTiltSpeed);
  }

  public double getXTiltSpeed(){
    return tiltXMotor.get();
  }
}
