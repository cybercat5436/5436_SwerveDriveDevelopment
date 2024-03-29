// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.enums.WheelPosition;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

/** Add your docs here. */
public class SwerveSubsystem extends SubsystemBase{
    
    private ArrayList<SwerveModuleState> moduleStates = new ArrayList<>();
    private ArrayList<SwerveModule> swerveModules = new ArrayList<>();

    private final SwerveModule frontLeft = new SwerveModule(
        WheelPosition.FRONT_LEFT,
        Constants.RoboRioPortConfig.FRONT_LEFT_DRIVE,
        Constants.RoboRioPortConfig.FRONT_LEFT_TURN,
        false,
        false,
        Constants.RoboRioPortConfig.ABSOLUTE_ENCODER_FRONT_LEFT,
        Constants.RoboRioPortConfig.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        true,
        IdleMode.kCoast,
        IdleMode.kCoast);

    private final SwerveModule frontRight = new SwerveModule(
        WheelPosition.FRONT_RIGHT,
        Constants.RoboRioPortConfig.FRONT_RIGHT_DRIVE,
        Constants.RoboRioPortConfig.FRONT_RIGHT_TURN,
        false,
        false,
        Constants.RoboRioPortConfig.ABSOLUTE_ENCODER_FRONT_RIGHT,
        Constants.RoboRioPortConfig.kFrontRightDriveAbsoluteEncoderOffsetRad,
        true,
        IdleMode.kCoast,
        IdleMode.kCoast);

    private final SwerveModule backLeft = new SwerveModule(
        WheelPosition.BACK_LEFT,
        Constants.RoboRioPortConfig.BACK_LEFT_DRIVE,
        Constants.RoboRioPortConfig.BACK_LEFT_TURN,
        false,
        false,
        Constants.RoboRioPortConfig.ABSOLUTE_ENCODER_BACK_LEFT,
        Constants.RoboRioPortConfig.kBackLeftDriveAbsoluteEncoderOffsetRad,
        true,
        IdleMode.kCoast,
        IdleMode.kCoast);

    private final SwerveModule backRight = new SwerveModule(
        WheelPosition.BACK_RIGHT,
        Constants.RoboRioPortConfig.BACK_RIGHT_DRIVE,
        Constants.RoboRioPortConfig.BACK_RIGHT_TURN,
        false,
        false,
        Constants.RoboRioPortConfig.ABSOLUTE_ENCODER_BACK_RIGHT,
        Constants.RoboRioPortConfig.kBackRightDriveAbsoluteEncoderOffsetRad,
        true,
        IdleMode.kCoast,
        IdleMode.kCoast);

    //idk if this is the gyro we have 
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
     new Rotation2d(0), new Pose2d(0.0, 3.0, new Rotation2d(0)));
    private int loopCount = 0;

    public SwerveSubsystem(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start(); 

        // Initialize the moduleStates array
        moduleStates.add(new SwerveModuleState());
        moduleStates.add(new SwerveModuleState());
        moduleStates.add(new SwerveModuleState());
        moduleStates.add(new SwerveModuleState());
        
        // Initialize the swerveModules array
        this.swerveModules.add(this.frontLeft);
        this.swerveModules.add(this.frontRight);
        this.swerveModules.add(this.backLeft);
        this.swerveModules.add(this.backRight);
    }

    public double getHeading(){
    return Math.IEEEremainder(-(gyro.getAngle()), 360);
}

public void zeroTurningEncoders(){
    for(int x=0; x<4; x++){
        swerveModules.get(x).zeroTurningEncoder();
    }
}
public Rotation2d getRotation2d(){

    return Rotation2d.fromDegrees(getHeading());
}

public Pose2d getPose(){
    return odometry.getPoseMeters();
}

public void resetOdometry(Pose2d pose){
    odometry.resetPosition(pose, getRotation2d());
}

public void zeroHeading(){
    gyro.reset();
}

public void setModuleStates(SwerveModuleState[] desiredStates){
    moduleStates.clear();
    for(int i = 0; i < 4; i++){
        moduleStates.add(desiredStates[i]);
    }
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
    odometry.update(getRotation2d(), frontLeft.getState(), frontRight.getState(),backLeft.getState(), backRight.getState());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    
    SmartDashboard.putNumber("Loop Count: ", loopCount++);
    // DataLogManager.log(String.format("Loop count %d", loopCount));

    for (SwerveModule swerveModule: swerveModules){
        // SmartDashboard.putNumber(String.format("%s Angle", swerveModule.wheelPosition.name()), swerveModule.getAbsoluteEncoderRadians());
        SmartDashboard.putNumber(String.format("%s Angle", swerveModule.wheelPosition.name()), swerveModule.getAbsoluteEncoderRadians());
        SmartDashboard.putNumber(String.format("%s Turning Encoder", swerveModule.wheelPosition.name()), swerveModule.getTurningPosition());
        SmartDashboard.putNumber(String.format("%s Target Angle", swerveModule.wheelPosition.name()), swerveModule.getState().angle.getRadians());
    }
    // SmartDashboard.putNumber("FL Angle", frontLeft.getAbsoluteEncoderRadians());
    // SmartDashboard.putNumber("FL Turning Encoder", frontLeft.getTurningPosition());
    // SmartDashboard.putNumber("FR Angle", frontRight.getAbsoluteEncoderRadians());
    // SmartDashboard.putNumber("BL Angle", backLeft.getAbsoluteEncoderRadians());
    // SmartDashboard.putNumber("BR Angle", backRight.getAbsoluteEncoderRadians());
    // SmartDashboard.putNumber("BL Encoder Voltage", backLeft.getAbsoluteEncoder().getVoltage());
    SmartDashboard.putNumber("5V RobotController", RobotController.getCurrent5V());
    
    // SmartDashboard.putNumber("FL Target Angle", moduleStates.get(0).angle.getRadians());
    SmartDashboard.putNumber("Gyro", gyro.getAngle());
    SmartDashboard.putNumber("Mystery", getHeading());

    // DataLogManager.log(String.format("Back Left Encoder Voltage %f", backLeft.getAbsoluteEncoder().getVoltage()));
    // DataLogManager.log(String.format("Back Right Encoder Voltage %f", backRight.getAbsoluteEncoder().getVoltage()));


    // DataLogManager.log(String.format("Back left Angle %f", backLeft.getAbsoluteEncoderRadians()));
    // DataLogManager.log(String.format("Back right Angle %f", backRight.getAbsoluteEncoderRadians()));
    // DataLogManager.log(String.format("Front left Angle %f", frontLeft.getAbsoluteEncoderRadians()));
    // DataLogManager.log(String.format("Front right Angle %f", frontRight.getAbsoluteEncoderRadians()));

    // DataLogManager.log(String.format("Voltage %f", RobotController.getCurrent5V()));
}

@Override
public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("FL Power", () -> frontLeft.getDriveVelocity(), null);
    builder.addDoubleProperty("FR Power", () -> frontRight.getDriveVelocity(), null);
    builder.addDoubleProperty("BL Power", () -> backLeft.getDriveVelocity(), null);
    builder.addDoubleProperty("BR Power", () -> backRight.getDriveVelocity(), null);
}

}


