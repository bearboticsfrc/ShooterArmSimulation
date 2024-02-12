// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Arm implements AutoCloseable {
  // The P gain for the PID controller that drives this arm.
  private double armKp = Constants.kDefaultArmKp;
  private double armKi = Constants.kDefaultArmKi;
  private double armKd = Constants.kDefaultArmKd;
  private double armSetpointDegrees = Constants.kDefaultArmSetpointDegrees;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor armGearbox = DCMotor.getNEO(2);

  // Standard classes for controlling our arm
  private final PIDController pidcontroller = new PIDController(armKp, armKi, armKd);
  private final Encoder encoder =
      new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
  private final PWMSparkMax motor = new PWMSparkMax(Constants.kMotorPort);

  private double feedForwardKs = Constants.kFeedForwardKs;
  private double feedForwardKg = Constants.kFeedForwardKg;
  private double feedForwardKv = Constants.kFeedForwardKv;
  private double feedForwardKa = Constants.kFeedForwardKa;


  private ArmFeedforward armFeedforward = new ArmFeedforward(feedForwardKs, feedForwardKg, feedForwardKv, feedForwardKa);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          armGearbox,
          Constants.kArmReduction,
          SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),
          Constants.kArmLength,
          Constants.kMinAngleRads,
          Constants.kMaxAngleRads,
          true,
          0,
          VecBuilder.fill(Constants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
          );
  private final EncoderSim encoderSim = new EncoderSim(encoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d armTower =
      armPivot.append(new MechanismLigament2d("ArmTower", 10, -90));
  private final MechanismLigament2d arm =
      armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  /** Subsystem constructor. */
  public Arm() {
    encoder.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);
    pidcontroller.setIZone(0.2);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", mech2d);
    armTower.setColor(new Color8Bit(Color.kBlue));

    // Set the Arm position setpoint and P constant to Preferences if the keys don't already exist
    Preferences.initDouble(Constants.kArmPositionKey, armSetpointDegrees);
    Preferences.initDouble(Constants.kArmPKey, armKp);
    Preferences.initDouble(Constants.kArmIKey, armKi);
    Preferences.initDouble(Constants.kArmDKey, armKd);
  
    Preferences.initDouble(Constants.kArmSKey, feedForwardKs);
    Preferences.initDouble(Constants.kArmGKey, feedForwardKg);
    Preferences.initDouble(Constants.kArmVKey, feedForwardKv);
    Preferences.initDouble(Constants.kArmAKey, feedForwardKa);
  }

  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    armSim.setInput(motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    encoderSim.setDistance(armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

    SmartDashboard.putNumber("CurrentDraw", armSim.getCurrentDrawAmps());
  }

  /** Load setpoint and kP from preferences. */
  public void loadPreferences() {
    // Read Preferences for Arm setpoint and kP on entering Teleop
    armSetpointDegrees = Preferences.getDouble(Constants.kArmPositionKey, armSetpointDegrees);

    if (armKp != Preferences.getDouble(Constants.kArmPKey, armKp) ||
    armKi != Preferences.getDouble(Constants.kArmIKey, armKi)||
    armKd != Preferences.getDouble(Constants.kArmDKey, armKd)) {
      armKp = Preferences.getDouble(Constants.kArmPKey, armKp);
      armKi = Preferences.getDouble(Constants.kArmIKey, armKi);
      armKd = Preferences.getDouble(Constants.kArmDKey, armKd);
      pidcontroller.setP(armKp);
      pidcontroller.setI(armKi);
      pidcontroller.setD(armKd);
    }

    if (feedForwardKs != Preferences.getDouble(Constants.kArmSKey, feedForwardKs) ||
        feedForwardKg != Preferences.getDouble(Constants.kArmGKey, feedForwardKg) ||
        feedForwardKv != Preferences.getDouble(Constants.kArmVKey, feedForwardKv) ||
        feedForwardKa != Preferences.getDouble(Constants.kArmAKey, feedForwardKa)) {
      feedForwardKs = Preferences.getDouble(Constants.kArmSKey, feedForwardKs);
      feedForwardKg = Preferences.getDouble(Constants.kArmGKey, feedForwardKg);
      feedForwardKv = Preferences.getDouble(Constants.kArmVKey, feedForwardKv);
      feedForwardKa = Preferences.getDouble(Constants.kArmAKey, feedForwardKa);
    }
 
  armFeedforward = new ArmFeedforward(feedForwardKs, feedForwardKg, feedForwardKv, feedForwardKa);
}
  

  /** Run the control loop to reach and maintain the setpoint from the preferences. */
  public void reachSetpoint(double setpoint) {
    armSetpointDegrees = setpoint;
    double pidOutput =
        pidcontroller.calculate(
            encoder.getDistance(), Units.degreesToRadians(armSetpointDegrees));
    
    double feedforward = armFeedforward.calculate(Math.toRadians(armSetpointDegrees), encoder.getRate());

    double armVoltage = MathUtil.clamp(feedforward + pidOutput, -1.0, 1.0) * 12.0;

    motor.setVoltage(armVoltage);
  }

  public boolean atSetPoint() {
    return Math.abs(Math.toDegrees(encoder.getDistance())-armSetpointDegrees) < 0.5;
  }

  public void stop() {
    motor.set(0.0);
  }

  @Override
  public void close() {
    motor.close();
    encoder.close();
    mech2d.close();
    armPivot.close();
    pidcontroller.close();
    arm.close();
  }
}
