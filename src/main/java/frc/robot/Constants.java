// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = .5;
    }

    public class RoboRioPortConfig{
        public static final int PDP = 0;

        public static final int FRONT_LEFT_DRIVE = 1;
        public static final int FRONT_RIGHT_DRIVE = 2;
        public static final int BACK_LEFT_DRIVE = 3;
        public static final int BACK_RIGHT_DRIVE = 4;
        public static final int FRONT_LEFT_TURN = 5;
        public static final int FRONT_RIGHT_TURN = 6;
        public static final int BACK_LEFT_TURN = 7;
        public static final int BACK_RIGHT_TURN = 8;
    
        public static final int ABSOLUTE_ENCODER_FRONT_LEFT = 0;
        public static final int ABSOLUTE_ENCODER_FRONT_RIGHT = 1;
        public static final int ABSOLUTE_ENCODER_BACK_LEFT = 2;
        public static final int ABSOLUTE_ENCODER_BACK_RIGHT = 3;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.26;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 1.87;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.66;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -.58f;
        


        //Constants from manufacturer
        public static final double DRIVE_MOTOR_GEAR_RATIO = 8.31;
    
    }

    public static class DriveConstants{
        /**we use the constants to make them a 1/4 of max speed
        because otherwise it would be too fast to control that is why making these numbers slower makes the 
        robot faster**/
        public static final double kTeleDriveMaxSpeedMetersPerSecond = 8;
        public static final double kPhysicalMaxSpeedMetersPerSecond = 2;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;


        public static final double kTrackWidth = Units.inchesToMeters(19);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(23.5);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    }

    public static class OIConstants{
        public static final double K_DEADBAND = .15;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

}
