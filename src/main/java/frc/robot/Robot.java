// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Arm;

/** This is a sample program to demonstrate the use of arm simulation with existing code. */
public class Robot extends TimedRobot {
  private final Arm arm = new Arm();
  private final Joystick joystick = new Joystick(Constants.kJoystickPort);

  private Timer timer = new Timer();
  @Override
  public void robotInit() {}

  @Override
  public void simulationPeriodic() {
    arm.simulationPeriodic();
  }

  @Override
  public void teleopInit() {
    arm.loadPreferences();
  }

  double positions[] = { 0.0, 15.0, 0.0, 25.0, 0.0, 35.0, 0.0, 45.0, 0.0, 55.0, 0.0 };
  int positionIndex = -1;

  boolean achievingSetPoint = false; 

  @Override
  public void teleopPeriodic() {
    if (joystick.getTriggerPressed()) {
      timer.reset();
      timer.start();
      achievingSetPoint = true;
      positionIndex++;
      if (positionIndex >= positions.length) {
        positionIndex = 0;
      }
      System.out.println("setpoint = " + positions[positionIndex]);
    }

    if (positionIndex >= 0 && positionIndex < positions.length) {
      // Here, we run PID control like normal.
      arm.reachSetpoint(positions[positionIndex]);
    } else {
      // Otherwise, we disable the motor.
      arm.stop();
    }

    if (arm.atSetPoint() && achievingSetPoint) {
      double time = timer.get();
      System.out.println(String.format("Time to set point: %.2f", time ));
      achievingSetPoint = false;
    }
  }

  @Override
  public void close() {
    arm.close();
    super.close();
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    arm.stop();
  }
}
