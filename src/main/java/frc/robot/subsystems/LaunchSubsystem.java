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

  private final PWMVictorSPX launchTopMotor = new PWMVictorSPX(0);
  private final PWMVictorSPX launchBottomMotor = new PWMVictorSPX(1);
  private final PWMVictorSPX pushMotor = new PWMVictorSPX(2);
  private final MotorControllerGroup launchMotors = new MotorControllerGroup(launchTopMotor, launchBottomMotor);
  private final DigitalInput ballCheckSwitch = new DigitalInput(0);
  private final Encoder wheelEncoder = new Encoder(1, 2);
  private final Encoder distanceEncoder = new Encoder(3, 4);
  public Timer wheelVelTimer = new Timer();

  public LaunchSubsystem() {
    double launchWheelRadius = 2;
    double distWheelRadius = 1;

    launchBottomMotor.setInverted(true);
    wheelEncoder.setDistancePerPulse(Math.PI * launchWheelRadius / 360);
    distanceEncoder.setDistancePerPulse(Math.PI * distWheelRadius / 360);
  }

  public boolean getCheckSwitchState(){
    return ballCheckSwitch.get();
  }

  public void setLauncherSpeed(double speed) {
    launchMotors.set(speed);
  }

  public void setPushSpeed(double speed) {
    pushMotor.set(speed);
  }

  public double getEncoderDistance() {
    System.out.println(distanceEncoder.getDistance());
    return distanceEncoder.getDistance();
  }

  public double getWheelVelocity() throws InterruptedException {
    System.out.println("get velocity start");
    wheelVelTimer.start();
    wheelEncoder.reset();
    //wait(100);
    wheelVelTimer.stop();
    System.out.println("get velocity end");
    return wheelEncoder.getDistance() / wheelVelTimer.get();
  }

  public double GetVelocity(double distance, double angle) {
    return Math.sqrt((32.171916 * distance) / Math.sin(2 * angle));
  }
}
