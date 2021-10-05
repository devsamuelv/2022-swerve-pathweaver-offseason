// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {}
  TalonFX LeftMotor = new TalonFX(Constants.SHOOTER_LEFT_MOTOR_ID);
  TalonFX RightMotor = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR_ID);
  public boolean ReachedVelocity = false;

  public void shoot(double draw_power) {
    RightMotor.setInverted(true);
    LeftMotor.set(ControlMode.Velocity, draw_power);
    RightMotor.set(ControlMode.Velocity, draw_power);
  }


  @Override
  public void periodic() {
    if (LeftMotor.getSelectedSensorVelocity() >= Constants.SHOOTER_TARGET_VELOCITY && RightMotor.getSelectedSensorVelocity() >= Constants.SHOOTER_TARGET_VELOCITY) {
      ReachedVelocity = true;
    }
  }
}
