// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaunchSubsystem extends SubsystemBase {

  private PWMVictorSPX launchTopMotor = new PWMVictorSPX(0);
  private PWMVictorSPX launchBottomMotor = new PWMVictorSPX(1);
  private PWMVictorSPX pushMotor = new PWMVictorSPX(2);
  //private PWMVictorSPX tiltXMotor = new PWMVictorSPX(3); **Motor Used With Vision**
  private PWMVictorSPX tiltYMotor = new PWMVictorSPX(4);
  private MotorControllerGroup launchMotors = new MotorControllerGroup(launchTopMotor, launchBottomMotor);
  public DigitalInput ballCheckSwitch = new DigitalInput(0);
  public DigitalInput tiltGroundSwitch = new DigitalInput(1);
  private Encoder tiltYEncoder = new Encoder(2, 3);

  public LaunchSubsystem() {
    launchBottomMotor.setInverted(true);
    tiltYEncoder.setDistancePerPulse(Math.PI/360);
  }

  public void setLauncherSpeed(double speed) {
    launchMotors.set(speed);
  }

  public void setPushSpeed(double speed) {
    pushMotor.set(speed);
  }

  public void setYTiltSpeed(double yTiltSpeed) {
    tiltYMotor.set(-yTiltSpeed);
  }

  public void resetEncoder() {
    tiltYEncoder.reset();
  }

  public double getEncoderRotation() {
    double dist = tiltYEncoder.getDistance();
    return dist/(Math.PI * 2) * 360;
  }
}
