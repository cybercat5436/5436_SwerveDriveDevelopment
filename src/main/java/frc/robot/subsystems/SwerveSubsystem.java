// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

/** Add your docs here. */
public class SwerveSubsystem extends SubsystemBase{


        
    private final SwerveModule frontLeft = new SwerveModule(
        Constants.RoboRioPortConfig.FRONT_LEFT_DRIVE,
        Constants.RoboRioPortConfig.FRONT_LEFT_TURN,
        false,
        false,
        Constants.RoboRioPortConfig.ABSOLUTE_ENCODER_FRONT_LEFT,
        Constants.RoboRioPortConfig.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        false);

    private final SwerveModule frontRight = new SwerveModule(
            Constants.RoboRioPortConfig.FRONT_RIGHT_DRIVE,
            Constants.RoboRioPortConfig.FRONT_RIGHT_TURN,
            false,
            false,
            Constants.RoboRioPortConfig.ABSOLUTE_ENCODER_FRONT_RIGHT,
            Constants.RoboRioPortConfig.kFrontRightDriveAbsoluteEncoderOffsetRad,
            false);

    private final SwerveModule backLeft = new SwerveModule(
            Constants.RoboRioPortConfig.BACK_LEFT_DRIVE,
            Constants.RoboRioPortConfig.BACK_LEFT_TURN,
            false,
            false,
            Constants.RoboRioPortConfig.ABSOLUTE_ENCODER_BACK_LEFT,
            Constants.RoboRioPortConfig.kBackLeftDriveAbsoluteEncoderOffsetRad,
            false);

    private final SwerveModule backRight = new SwerveModule(
            Constants.RoboRioPortConfig.BACK_RIGHT_DRIVE,
            Constants.RoboRioPortConfig.BACK_RIGHT_TURN,
            false,
            false,
            Constants.RoboRioPortConfig.ABSOLUTE_ENCODER_BACK_RIGHT,
            Constants.RoboRioPortConfig.kBackRightDriveAbsoluteEncoderOffsetRad,
            false);

    //idk if this is the gyro we have 
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    public SwerveSubsystem(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }
    public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
}

public Rotation2d getRotation2d(){

    return Rotation2d.fromDegrees(getHeading());
}

public void zeroHeading(){
    gyro.reset();
}

public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);

}

public void stopModules(){

}
@Override
public void periodic() {

    SmartDashboard.putNumber("FL Angle", frontLeft.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("FR Angle", frontRight.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("BL Angle", backLeft.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("BR Angle", backRight.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("BL Left Encoder Voltage", backLeft.getAbsoluteEncoder().getVoltage());
    SmartDashboard.putNumber("5V RobotController", RobotController.getCurrent5V());
    


}

}


