// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  TalonFX shooterLeft = new TalonFX(0);
  TalonFX shooterRight = new TalonFX(0);
  Spark intake = new Spark(0);

  double targetVel = 1000;

  /** Creates a new Shooter. */
  public Shooter() {
  }

  public void revUp() {
    this.shooterLeft.set(ControlMode.Velocity, targetVel);
    this.shooterRight.set(ControlMode.Velocity, targetVel);

    if (shooterLeft.getSelectedSensorVelocity() >= (targetVel - 100)
        && shooterRight.getSelectedSensorVelocity() >= (targetVel - 100)) {
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
