// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  TalonFX shooterLeft = new TalonFX(0);
  TalonFX shooterRight = new TalonFX(0);
  Spark intake = new Spark(0);
  BooleanSupplier fire;

  /** Creates a new Shooter. */
  public Shooter(BooleanSupplier fire) {
    this.fire = fire;
  }

  public void revUp() {
    this.shooterLeft.set(ControlMode.Velocity, Constants.SHOOTER_TARGET_VELOCITY);
    this.shooterRight.set(ControlMode.Velocity, Constants.SHOOTER_TARGET_VELOCITY);

    if (shooterLeft.getSelectedSensorVelocity() >= (Constants.SHOOTER_TARGET_VELOCITY - 100)
        && shooterRight.getSelectedSensorVelocity() >= (Constants.SHOOTER_TARGET_VELOCITY - 100)
        && fire.getAsBoolean()) {
      intake.set(-0.8);
    } else {
      intake.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
