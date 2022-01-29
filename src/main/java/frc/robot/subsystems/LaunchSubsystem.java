// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
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
  private Encoder wheelEncoder = new Encoder(4, 5);
  private Encoder distanceEncoder = new Encoder(6, 7);
  public Timer wheelVelTimer = new Timer();

  public LaunchSubsystem() {
    double launchWheelRadius = 2;
    double distWheelRadius = 1;

    launchBottomMotor.setInverted(true);
    tiltYEncoder.setDistancePerPulse(Math.PI/360);
    wheelEncoder.setDistancePerPulse(Math.PI * launchWheelRadius / 360);
    distanceEncoder.setDistancePerPulse(Math.PI * distWheelRadius/ 12);
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

  public void resetYTiltEncoder() {
    tiltYEncoder.reset();
  }

  public double getTiltEncoderRotation() {
    double dist = tiltYEncoder.getDistance();
    return dist/(Math.PI * 2) * 360;
  }

  public double getDistance() {
    return distanceEncoder.getDistance();
  }

  public double getWheelVelocity() throws InterruptedException {
    wheelVelTimer.start();
    wheelEncoder.reset();
    wait(100);
    wheelVelTimer.stop();
    return wheelEncoder.getDistance() / wheelVelTimer.get();
  }

  public double requiredAngle(double velocity, double distance, double height, double gravity) {
    double negB = 2 * Math.pow(velocity, 2) * distance;
    double disc = (4 * Math.pow(velocity, 4) * Math.pow(distance, 2)) - (4 * Math.pow(velocity, 2) * height * -gravity * Math.pow(distance, 2));
    double divisor = 4 * Math.pow(velocity, 2) * height;
    return (negB + Math.sqrt(disc * (Math.PI/4))) / divisor;
  }
}
