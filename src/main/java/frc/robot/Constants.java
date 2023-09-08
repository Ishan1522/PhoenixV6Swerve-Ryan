// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class HardwareConstants {
    public static final String CANIVORE_STRING = "Canivore 1";
    // public static final String CANIVORE_CAN_BUS_STRING = "rio";
    public static final String RIO_STRING = "rio";

    public static final double FALCON_ENCODER_RESOLUTION = 2048.0;
    public static final double CANCODER_RESOLUTION = 4096.0; 

    public static final double MIN_FALCON_DEADBAND = 0.001;

    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;

    public static final int TIMEOUT_MS = 30;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
   public static final class DriveConstants {

    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(22.25);
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(28.5);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Rear Left
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Rear Right
    );

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 3;
    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 4;

    public static final int FRONT_LEFT_TURN_MOTOR_ID = 5;
    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 6;
    public static final int REAR_LEFT_TURN_MOTOR_ID = 7;
    public static final int REAR_RIGHT_TURN_MOTOR_ID = 8;

    public static final int FRONT_LEFT_CANCODER_ID = 11;
    public static final int FRONT_RIGHT_CANCODER_ID = 12;
    public static final int REAR_LEFT_CANCODER_ID = 13;
    public static final int REAR_RIGHT_CANCODER_ID = 14;

    public static final double FRONT_LEFT_ZERO_ANGLE = 169.716796875;
    public static final double FRONT_RIGHT_ZERO_ANGLE = -76.46484375;
    public static final double REAR_LEFT_ZERO_ANGLE = 46.58203125;
    public static final double REAR_RIGHT_ZERO_ANGLE = -78.57421875 + 90;

    public static final boolean FRONT_LEFT_CANCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_CANCODER_REVERSED = false;
    public static final boolean REAR_LEFT_CANCODER_REVERSED = false;
    public static final boolean REAR_RIGHT_CANCODER_REVERSED = false;
    
    public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = true; 
    public static final boolean REAR_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean REAR_RIGHT_DRIVE_ENCODER_REVERSED = true;
    
    public static final double TURN_S = 0.77;
    public static final double TURN_V = 0.75;
    public static final double TURN_A = 0;
    public static final double MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND = 2;

    public static final double MAX_SPEED_METERS_PER_SECOND = 1; //4.5

    public static final double FACEFORWARD_P = 0.015;

    // TODO: Remove this if we don't end up doing auto balance
    public static final class BalanceConstants {
      public static final double BALANCE_P = 0.07;
      public static final double BALANCE_I = 0;
      public static final double BALANCE_D = 0;
      
      public static final double ORIENTATION_ERROR_CONSIDERED_ORIENTED = 2.5; // +/- degrees
      public static final double BALANCE_ERROR_CONSIDERED_BALANCED = 2.4; // +/- degrees

      public static final double INITIAL_SPEED = 0.7;
      public static final double BALANCE_ERROR_INIT_DEGREES = 10;
      public static final double BALANCE_ERROR_NEAR_BALANCED = 3;
    }
   }
  public static class ModuleConstants {
    public static final double DRIVE_GEAR_RATIO = 7.36;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_TO_METERS = 
      WHEEL_CIRCUMFERENCE_METERS / (DRIVE_GEAR_RATIO);
    public static final double DRIVE_TO_METERS_PER_SECOND =
      (10 * WHEEL_CIRCUMFERENCE_METERS) / (DRIVE_GEAR_RATIO);
    public static final double TURN_GEAR_RATIO = 16.0;

    public static final double TURN_P = 0; // 7/2048
    public static final double TURN_I = 0;
    public static final double TURN_D = 0;

    public static final double TURN_S = 0.77;
    public static final double TURN_V = 0.75;
    public static final double TURN_A = 0;
    public static final double MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND = 2;
    public static final double MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 4;
    public static final TrapezoidProfile.Constraints TURN_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
        MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND,
        MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED
      );

    public static final double DRIVE_P = 0.3/2048;
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0;
    public static final double DRIVE_S = 0.29155 / 12.0;
    public static final double DRIVE_V = 2.3621 / 12.0;
    public static final double DRIVE_A = 0.72606 / 12.0;
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    // Angle Motor PID Values
    public static final double angleKP = 40.0;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.5;

    // Drive Motor PID Values
    public static final double driveKP = 2.0;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    // Drive Motor Characterization Values
    public static final double driveKS = 0.2;
    public static final double driveKV = 2.0;

    // Angle Encoder Invert
    public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

    // Motor Inverts
    public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;

    // Neutral Modes
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final double driveGearRatio = 7.36;
    public static final double angleGearRatio = 16;
    public static final double rotationsPerMeter = driveGearRatio / wheelCircumference;
  }
  
  public static final class JoystickConstants {

    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;
    public static final int BUTTON_BOARD_1_ID = 2;
    public static final int BUTTON_BOARD_2_ID = 3;

    public static final int DRIVER_LEFT_STICK_X = 0;
    public static final int DRIVER_LEFT_STICK_Y = 1;
    public static final int DRIVER_RIGHT_STICK_X = 4;
    public static final int DRIVER_RIGHT_STICK_Y = 5;
    public static final int DRIVER_X_BUTTON_ID = 3;
    public static final int DRIVER_A_BUTTON_ID = 1;
    public static final int DRIVER_B_BUTTON_ID = 2;
    public static final int DRIVER_Y_BUTTON_ID = 4;
    public static final int DRIVER_LEFT_BUMPER_ID = 5;
    public static final int DRIVER_RIGHT_BUMPER_ID = 6;
    public static final int DRIVER_LEFT_TRIGGER_ID = 2;
    public static final int DRIVER_RIGHT_TRIGGER_ID = 3;
    public static final int DRIVER_BACK_BUTTON_ID = 7;
    public static final int DRIVER_START_BUTTON_ID = 8;
    public static final int DRIVER_LEFT_STICK_PRESS_ID = 9;
    public static final int DRIVER_RIGHT_STICK_PRESS_ID = 10;

    public static final int OPERATOR_LEFT_STICK_X = 0;
    public static final int OPERATOR_LEFT_STICK_Y = 1;
    public static final int OPERATOR_RIGHT_STICK_X = 4;
    public static final int OPERATOR_RIGHT_STICK_Y = 5;

    public static final int OPERATOR_X_BUTTON_ID = 3;
    public static final int OPERATOR_A_BUTTON_ID = 1;
    public static final int OPERATOR_B_BUTTON_ID = 2;
    public static final int OPERATOR_Y_BUTTON_ID = 4;
    public static final int OPERATOR_LEFT_BUMPER_ID = 5;
    public static final int OPERATOR_RIGHT_BUMPER_ID = 6;
    public static final int OPERATOR_LEFT_TRIGGER_ID = 2;
    public static final int OPERATOR_RIGHT_TRIGGER_ID = 3;
    public static final int OPERATOR_BACK_BUTTON_ID = 7;
    public static final int OPERATOR_START_BUTTON_ID = 8;
    public static final int OPERATOR_LEFT_STICK_PRESS_ID = 9;
    public static final int OPERATOR_RIGHT_STICK_PRESS_ID = 10;

    public static final int LEFT_DPAD_ID = 270;
    public static final int UP_DPAD_ID = 0;
    public static final int RIGHT_DPAD_ID = 90;
    public static final int DOWN_DPAD_ID = 180;

    // Top Six Buttons

    /** Z axis */
    public static final int BIG_BUTTON_1 = -1;
    /** Z axis */
    public static final int BIG_BUTTON_2 = 1;
    /** X axis */
    public static final int BIG_BUTTON_3 = -1;
    /** X axis */
    public static final int BIG_BUTTON_4 = 1;
    /** Y axis */
    public static final int BIG_BUTTON_5 = -1;
    /** Y axis */
    public static final int BIG_BUTTON_6 = 1;

    // Joystick 1 Autoplace Buttons
    public static final int BUTTON_1 = 1;
    public static final int BUTTON_2 = 2;
    public static final int BUTTON_3 = 3;
    public static final int BUTTON_4 = 4;
    public static final int BUTTON_5 = 5;
    public static final int BUTTON_6 = 6;
    public static final int BUTTON_7 = 7;
    public static final int BUTTON_8 = 8;
    public static final int BUTTON_9 = 9;
    public static final int BUTTON_10 = 10;
    public static final int BUTTON_11 = 11;
    public static final int BUTTON_12 = 1; // Z axis going forwards
    public static final int BUTTON_13 = 0; // POV Joystick 1
    public static final int BUTTON_14 = 180; // POV Joystick 1
    public static final int BUTTON_15 = 270; // POV Joystick 1
    public static final int BUTTON_16 = 90; // POV Joystick 1

    // Joystick 2 Autoplace Buttons
    public static final int BUTTON_17 = 1;
    public static final int BUTTON_18 = 2;
    public static final int BUTTON_19 = 3;
    public static final int BUTTON_20 = 4;
    public static final int BUTTON_21 = 5;
    public static final int BUTTON_22 = 6;
    public static final int BUTTON_23 = 7;
    public static final int BUTTON_24 = 8;
    public static final int BUTTON_25 = 9;
    public static final int BUTTON_26 = 10;
    public static final int BUTTON_27 = 11;
  }

}
