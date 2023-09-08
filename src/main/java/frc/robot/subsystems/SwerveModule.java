// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  private MotionMagicVoltage turnVoltage;
  private VelocityVoltage driveVoltage;

  private BaseStatusSignal[] signals;
  private StatusSignal<Double> drivePosition;
  private StatusSignal<Double> driveVelocity;
  private StatusSignal<Double> anglePosition;
  private StatusSignal<Double> angleVelocity;

  private final ProfiledPIDController turnPIDController = new ProfiledPIDController(
    ModuleConstants.TURN_P, 
    ModuleConstants.TURN_I, 
    ModuleConstants.TURN_D, 
    ModuleConstants.TURN_CONSTRAINTS
  );

  private TalonFX driveMotor;
  private TalonFX turnMotor;
  private CANcoder turnEncoder;

  private String name;

  /** Creates a new SwerveModule. */
  public SwerveModule(String name, int driveMotorID, int turnMotorID, int turnEncoderID, double angleOffset, boolean driveInverted, boolean turnEncoderInverted) {
    this.name = name;

    driveMotor = new TalonFX(driveMotorID, HardwareConstants.CANIVORE_STRING);
    turnMotor = new TalonFX(turnMotorID, HardwareConstants.CANIVORE_STRING);
    turnEncoder = new CANcoder(turnEncoderID, HardwareConstants.CANIVORE_STRING);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.Slot0.kP = ModuleConstants.DRIVE_P;
    driveConfig.Slot0.kI = ModuleConstants.DRIVE_I;
    driveConfig.Slot0.kD = ModuleConstants.DRIVE_D;
    driveConfig.Slot0.kS = ModuleConstants.DRIVE_S;
    driveConfig.Slot0.kI = ModuleConstants.DRIVE_I;
    driveConfig.Slot0.kV = ModuleConstants.DRIVE_V;
    driveConfig.MotorOutput.DutyCycleNeutralDeadband = 10 * HardwareConstants.MIN_FALCON_DEADBAND;

    driveConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.driveContinuousCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = ModuleConstants.driveEnableCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentThreshold = ModuleConstants.drivePeakCurrentLimit;
    driveConfig.CurrentLimits.SupplyTimeThreshold = ModuleConstants.drivePeakCurrentDuration;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveMotor.getConfigurator().apply(driveConfig);
    driveMotor.setRotorPosition(0.0);
    driveMotor.setInverted(driveInverted);

    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.Slot0.kP = ModuleConstants.TURN_P;
    turnConfig.Slot0.kI = ModuleConstants.TURN_I;
    turnConfig.Slot0.kD = ModuleConstants.TURN_D;
    turnConfig.Slot0.kS = ModuleConstants.TURN_S;
    turnConfig.Slot0.kV = ModuleConstants.TURN_V;
    turnConfig.MotorOutput.DutyCycleNeutralDeadband = 10 * HardwareConstants.MIN_FALCON_DEADBAND;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // turnMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND, HardwareConstants.TIMEOUT_MS);
    // turnMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1), HardwareConstants.TIMEOUT_MS);
    // turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1), HardwareConstants.TIMEOUT_MS);
    // turnMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    // turnMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);  

    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    turnEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    turnEncoderConfig.MagnetSensor.MagnetOffset = angleOffset;
    turnEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    turnEncoder.getConfigurator().apply(turnEncoderConfig);

    TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();
    driveMotorConfiguration.Slot0.kS = ModuleConstants.DRIVE_S;
    driveMotorConfiguration.Slot0.kV = ModuleConstants.DRIVE_V;
    driveMotorConfiguration.Slot0.kP = ModuleConstants.DRIVE_P;
    driveMotorConfiguration.Slot0.kI = ModuleConstants.DRIVE_I;
    driveMotorConfiguration.Slot0.kD = ModuleConstants.DRIVE_D;
    driveMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotorConfiguration.MotorOutput.Inverted = driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveMotorConfiguration.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND * 10.;
    driveMotorConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    driveMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    driveMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    driveMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 65;
    driveMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotorConfiguration.CurrentLimits.SupplyTimeThreshold = 0.1;
    driveMotor.getConfigurator().apply(driveMotorConfiguration);
    
    TalonFXConfiguration turnMotorConfiguration = new TalonFXConfiguration();
    turnMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turnMotorConfiguration.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND * 10.;
    turnMotorConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    turnMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    turnMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    turnMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 65;
    turnMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnMotorConfiguration.CurrentLimits.SupplyTimeThreshold = 0.1;
    turnMotorConfiguration.Slot0.kS = ModuleConstants.TURN_S;
    turnMotorConfiguration.Slot0.kV = ModuleConstants.TURN_V;
    turnMotor.getConfigurator().apply(turnMotorConfiguration);

    

    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    anglePosition = turnMotor.getPosition();
    angleVelocity = turnMotor.getVelocity();

    signals = new BaseStatusSignal[] {
      drivePosition,
      driveVelocity,
      anglePosition,
      angleVelocity,
    };
  }
  public SwerveModulePosition getPosition() {
    
    //gets the "latency compensated value" for the rotationz
    double driveSignals = BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
    double angleSignals = BaseStatusSignal.getLatencyCompensatedValue(anglePosition, angleVelocity);

    double distance = driveSignals / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;
    // internalState.distanceMeters = distance;
    Rotation2d angle = Rotation2d.fromRotations(angleSignals);
    // internalState.angle = angle;
    
    return new SwerveModulePosition(distance, angle);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinnin further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(getAnglePosition()));

    // Converts meters per second to RPM
    double desiredDriveRPM = optimizedDesiredState.speedMetersPerSecond * 60
      * ModuleConstants.DRIVE_GEAR_RATIO / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

    // Converts RPM to rotations per 100ms
    double desiredDriveRPOHMS = desiredDriveRPM * 60. * 1000.;

    // Sets the drive motor's speed using the built in PID controller
    driveMotor.set(desiredDriveRPOHMS / 12.);
    
    double turnOutput = turnPIDController.calculate(getAnglePosition(), optimizedDesiredState.angle.getRotations());
    turnMotor.set(turnOutput);
  } 

  @Override
  public void periodic() {
    drivePosition.refresh();
    driveVelocity.refresh();
    anglePosition.refresh();
    angleVelocity.refresh();
    SmartDashboard.putNumber("angle position " + name, getAnglePosition());
    // This method will be called once per scheduler run
  }

  private double getAnglePosition() {
    return (anglePosition.getValue() / ModuleConstants.TURN_GEAR_RATIO) % 1;
  }
}
