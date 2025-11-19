// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  SparkMax motor = new SparkMax(22, MotorType.kBrushless); // constructing motor

  private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X); // constructing encoder
  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12; // encoder records in ticks, but we need it in feet.

  final double kP = 0.5;
  double setpoint;
  double error;

  public DriveSubsystem() {encoder.reset();}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double encoderPosition = encoder.get() * kDriveTick2Feet; // encoder position converted to feed
    error = setpoint - encoderPosition;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setMotorVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public void stop() {
    motor.setVoltage(0.0);
  }

  public void driveToDestination(double destination) {
    setpoint = destination;
    double output = kP * error;
    motor.setVoltage(output);
  }
}
