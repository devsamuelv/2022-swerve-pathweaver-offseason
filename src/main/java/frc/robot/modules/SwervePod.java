// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import javax.annotation.Nullable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.hal.EncoderJNI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;

/** Add your docs here. */
public class SwervePod {
  double kWheelRadius = 0.0508;
  int kEncoderResolution = 4096;

  double kModuleMaxAngularVelocity = Constants.kMaxAngularSpeed;
  double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  // tune the pid values for the robot
  PIDController m_drivePIDController = new PIDController(1, 0, 0);
  ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  TalonFX speedMotor;
  TalonFX angularMotor;
  CANCoder canCoder;

  public SwervePod(int speedMotorId, int angularMotorId, int canCoderId) {
    this.speedMotor = new TalonFX(speedMotorId);
    this.angularMotor = new TalonFX(angularMotorId);
    this.canCoder = new CANCoder(canCoderId);

    this.m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwervePod(int speedMotorId, int angularMotorId, int canCoderId, double kWheelRadius, int kEncoderResolution) {
    this.speedMotor = new TalonFX(speedMotorId);
    this.angularMotor = new TalonFX(angularMotorId);
    this.canCoder = new CANCoder(canCoderId);

    this.kWheelRadius = kWheelRadius;
    this.kEncoderResolution = kEncoderResolution;

    this.m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModuleState getState() {
    // if (int) motor.getHandle does not work try using the deviceId
    return new SwerveModuleState(EncoderJNI.getEncoderRate((int) speedMotor.getHandle()),
        new Rotation2d(Math.toRadians(canCoder.getPosition())));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        new Rotation2d(Math.toRadians(canCoder.getPosition())));

    // Calculate the drive output from the drive PID controller.
    // if (int) motor.getHandle does not work try using the deviceId
    final double driveOutput = m_drivePIDController.calculate(EncoderJNI.getEncoderRate((int) speedMotor.getHandle()),
        state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(canCoder.getPosition(), state.angle.getRadians());

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    speedMotor.set(ControlMode.Current, driveOutput + driveFeedforward);
    angularMotor.set(ControlMode.Current, turnOutput + turnFeedforward);
  }
}
