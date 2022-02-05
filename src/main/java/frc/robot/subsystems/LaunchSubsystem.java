// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaunchSubsystem extends SubsystemBase {

  private final WPI_TalonFX launchTopMotor;
  private final WPI_TalonFX launchBottomMotor;
  private final MotorControllerGroup launchMotors;
  public Timer wheelVelTimer = new Timer();

  public LaunchSubsystem() {
    System.out.println("Subsystem Created!");

    launchTopMotor = new WPI_TalonFX(0);
    launchBottomMotor = new WPI_TalonFX(1);
    launchMotors = new MotorControllerGroup(launchTopMotor, launchBottomMotor);
    launchTopMotor.setInverted(true);
  }

  public void setLauncherSpeed(double speed) {
    launchTopMotor.set(.3);
    launchBottomMotor.set(.85);
  }

  public double getLauncherSpeed() {
    return launchTopMotor.get();
  }

  public double getEncoderDistance() {
    //return distanceEncoder.getDistance();
    return 0;
  }

  public double getWheelVelocity() {
    /*System.out.println("get velocity start");
    wheelVelTimer.start();
    wheelEncoder.reset();
    //wait(100);
    wheelVelTimer.stop();
    System.out.println("get velocity end: " + wheelEncoder.getDistance() / wheelVelTimer.get());
    return wheelEncoder.getDistance() / wheelVelTimer.get();*/
    return 0;
  }

  public double GetVelocity(double distance, double angle) {
    return Math.sqrt((32.171916 * distance) / Math.sin(2 * angle));
  }
}
