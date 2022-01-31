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
  private MotorControllerGroup launchMotors = new MotorControllerGroup(launchTopMotor, launchBottomMotor);
  public DigitalInput ballCheckSwitch = new DigitalInput(0);
  private Encoder wheelEncoder = new Encoder(1, 2);
  private Encoder distanceEncoder = new Encoder(3, 4);
  public Timer wheelVelTimer = new Timer();

  public LaunchSubsystem() {
    double launchWheelRadius = 2;
    double distWheelRadius = 1;

    launchBottomMotor.setInverted(true);
    wheelEncoder.setDistancePerPulse(Math.PI * launchWheelRadius / 360);
    distanceEncoder.setDistancePerPulse(Math.PI * distWheelRadius/ 360);
  }

  public void setLauncherSpeed(double speed) {
    launchMotors.set(speed);
  }

  public void setPushSpeed(double speed) {
    pushMotor.set(speed);
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

  public double RequiredDistance(double velocity) {
    double negB = -Math.pow(velocity * Math.sqrt(2) / 2, 2);
    double disc = velocity * Math.sqrt(Math.pow(velocity * Math.sqrt(2) / 2, 2) - (4 * -4.9 * 10));
    return (negB - disc) / 9.8;
  }
}
