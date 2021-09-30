// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  Spark motor = new Spark(Constants.INTAKE_MOTOR_ID);

  /** Creates a new Intake. */
  public Intake() {
  }

  public void setSpeed(double _power) {
    motor.set(_power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
