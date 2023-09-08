// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  
  private final DoubleSupplier leftY, leftX, rightX;
  private final BooleanSupplier isFieldRelative;
 
   public DriveCommand(DriveSubsystem driveSubsystem,  DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, BooleanSupplier isFieldRelative) {
    
    this.driveSubsystem = driveSubsystem;
    this.leftY = leftY;
    this.leftX = leftX;
    this.rightX = rightX;
    this.isFieldRelative = isFieldRelative;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    driveSubsystem.drive(
      leftY.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
      leftX.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
      rightX.getAsDouble() * DriveConstants.MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND,
      isFieldRelative.getAsBoolean()
    );
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
