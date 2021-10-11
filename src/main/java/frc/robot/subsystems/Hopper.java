// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  CANSparkMax leftMotor = new CANSparkMax(0, MotorType.kBrushed);
  CANSparkMax rightMotor = new CANSparkMax(0, MotorType.kBrushed);
  BooleanSupplier isRunning = () -> false;

  public Hopper() {
  }

  public void setSpeed(double _speed) {
    this.rightMotor.set(_speed);
    this.leftMotor.set(_speed);

    if (_speed > 0.1 || _speed < -0.1) {
      isRunning = () -> true;
    } else if (_speed == 0) {
      isRunning = () -> false;
    }
  }

  public BooleanSupplier isRunning() {
    return this.isRunning;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
