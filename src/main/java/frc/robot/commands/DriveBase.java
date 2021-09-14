// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;

public class DriveBase extends CommandBase {
  private Base base;
  private DoubleSupplier speedX;
  private DoubleSupplier speedY;
  private DoubleSupplier rot;

  /** Creates a new DriveBase. */
  public DriveBase(Base base, DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier rot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.base = base;
    this.speedX = speedX;
    this.speedY = speedY;
    this.rot = rot;

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.base.Drive(this.speedX.getAsDouble(), this.speedY.getAsDouble(), rot.getAsDouble(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
