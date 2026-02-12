// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final double DRIVE_SPEED = 1.0;

    // motor ids 1-8
    public static final int[] kDriverIds = new int[8];
    // cancoder ids 9-12
    public static final int[] kEncoderIds = new int[4];
    // pigeon id 0
    public static final int kPigeonId = 0;
  }

  public static class IntakeConstants {
    public static final double INTAKE_SPEED = 1.0;
    
    // motor id 13-14
    public static final Integer kRollerMotorId = 13;
    public static final Boolean kRollerMotorInverted = false;
    public static final Integer kPivotMotorId = 14;
    public static final Boolean kPivotMotorInverted = false;
  }

  public static class HopperConstants {
    public static final double HOPPER_SPEED = 1.0;

    // motor id 15
    public static final Integer kHopperMotorId = 15;
    public static final Boolean kHopperMotorInverted = false; // edit inversions when doing configuration
  }

  public static class FeederConstants {
    public static final double FEEDER_SPEED = 1.0;

    // motor id 16
    public static final Integer kFeederMotorId = 16;
    public static final Boolean kFeederMotorInverted = false;
  }

  public static class TurretConstants {
    public static final double MAX_ONE_DIR_FOV = 90; // degrees
    public static final Translation3d turretTranslation = new Translation3d(-0.205, 0.0, 0.375); // (cranberry)

    // motor id 17
    public static final Integer kTurretMotorId = 17;
    public static final Boolean kTurretMotorInverted = false;
  }

  public static final class ShooterConstants {
    // motor id 18
    public static final Integer kShooterMotorId = 18;
    public static final Boolean kTurretMotorInverted = false;

    public static final Double kFlywheelDiameter = 4.0;
  }

  public static final class ClimberConstants {
    //motor id 19
    public static final Integer kClimberMotorId = 19;
    public static final Boolean kClimberMotorInverted = false;
  }

  public static final class SwerveConstants {
    public static final boolean IS_LIMELIGHT_ENABLED = true;
    // NEED TO CONFIGURE ENCODERS IN SWERVE CONFIG (deploy)
  }
}
