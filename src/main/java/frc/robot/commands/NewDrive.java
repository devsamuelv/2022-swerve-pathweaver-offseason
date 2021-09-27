// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NewBase;

public class NewDrive extends CommandBase {
  private final NewBase base;
  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier rotation;

  public NewDrive(NewBase base, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    this.base = base;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationXPercent = translationX.getAsDouble();
    double translationYPercent = translationY.getAsDouble();
    double translationRotPercent = rotation.getAsDouble();

    this.base.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationXPercent * NewBase.MAX_VELOCITY_METERS_PER_SECOND,
        translationYPercent * NewBase.MAX_VELOCITY_METERS_PER_SECOND,
        translationRotPercent * NewBase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, base.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
