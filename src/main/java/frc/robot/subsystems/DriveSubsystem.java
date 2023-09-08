// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final SwerveModule frontLeftSwerveModule, frontRightSwerveModule, rearLeftSwerveModule, rearRightSwerveModule;
  private final Gyro gyro;

  private double gyroOffset;

  public DriveSubsystem() {
    frontLeftSwerveModule = new SwerveModule(
      "FL",
      DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_LEFT_TURN_MOTOR_ID,
      DriveConstants.FRONT_LEFT_CANCODER_ID,
      DriveConstants.FRONT_LEFT_ZERO_ANGLE,
      DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
      DriveConstants.FRONT_LEFT_CANCODER_REVERSED
    );

    frontRightSwerveModule = new SwerveModule(
      "FR",
      DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_TURN_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_CANCODER_ID,
      DriveConstants.FRONT_RIGHT_ZERO_ANGLE,
      DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
      DriveConstants.FRONT_RIGHT_CANCODER_REVERSED
    );

    rearLeftSwerveModule = new SwerveModule(
      "RL",
      DriveConstants.REAR_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_LEFT_TURN_MOTOR_ID,
      DriveConstants.REAR_LEFT_CANCODER_ID,
      DriveConstants.REAR_LEFT_ZERO_ANGLE,
      DriveConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED,
      DriveConstants.REAR_LEFT_CANCODER_REVERSED
    );

    rearRightSwerveModule = new SwerveModule(
      "RR",
      DriveConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_RIGHT_TURN_MOTOR_ID,
      DriveConstants.REAR_RIGHT_CANCODER_ID,
      DriveConstants.REAR_RIGHT_ZERO_ANGLE,
      DriveConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED,
      DriveConstants.REAR_RIGHT_CANCODER_REVERSED
    );

    gyro = new AHRS(SPI.Port.kMXP);
  }

  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getRotation2d(true))
      : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    setModulePositions(swerveModuleStates);
    
  }


  public double getHeading() {
    return (-gyro.getAngle() + this.gyroOffset) % 360;
  }
  public Rotation2d getRotation2d() { return getRotation2d(false); };

  public Rotation2d getRotation2d(boolean fieldRelative) {
    if (fieldRelative) {
      return Rotation2d.fromDegrees((getHeading() + (DriverStation.getAlliance() == Alliance.Blue ? 0 : 180)) % 360);
    } else {
      return Rotation2d.fromDegrees(getHeading());
    }
  }

  public void setGyroOffset(double newGyroOffset) { gyroOffset = newGyroOffset % 360;}

  public void zeroHeading() {
    setGyroOffset((DriverStation.getAlliance() == Alliance.Blue ? 0 : 180));
    gyro.reset();
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeftSwerveModule.getPosition(),
      frontRightSwerveModule.getPosition(),
      rearLeftSwerveModule.getPosition(),
      rearRightSwerveModule.getPosition()
    };
  }
  public void setModulePositions(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeftSwerveModule.setDesiredState(desiredStates[0]);
    frontRightSwerveModule.setDesiredState(desiredStates[1]);
    rearLeftSwerveModule.setDesiredState(desiredStates[2]);
    rearRightSwerveModule.setDesiredState(desiredStates[3]);
  }
}
