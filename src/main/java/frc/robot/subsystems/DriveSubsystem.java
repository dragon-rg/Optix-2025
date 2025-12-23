// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  // public final SparkMax motor = new SparkMax(5, MotorType.kBrushless);

  LinearSystem<N1, N1, N1> drivePlant = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.09, 1);
  public final FlywheelSim motorSim = new FlywheelSim(drivePlant, DCMotor.getNEO(1), 1);
  public DriveSubsystem() {}

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
e   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Voltage", motorSim.getInputVoltage());

    // Logger.recordOutput("Voltage", motor.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setMotorVoltage(double volts) {
    motorSim.setInputVoltage(volts);
    // motor.setVoltage(volts);
  }

  public void stop() {
    motorSim.setInputVoltage(0.0);
    // motor.setVoltage(0.0);
  }
}
