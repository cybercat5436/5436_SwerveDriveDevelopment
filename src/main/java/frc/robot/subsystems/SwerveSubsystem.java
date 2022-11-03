// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import javax.xml.namespace.QName;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;


import java.io.File;
import java.io.PrintWriter;
import java.util.Scanner;
import java.io.IOException;


/** Add your docs here. */
public class SwerveSubsystem extends SubsystemBase{

    private double frontLeftDriveAbsoluteEncoderOffsetRad;
    private double frontRightDriveAbsoluteEncoderOFfsetRad;
    private double backLeftDriveAbsoluteEncoderOffsetRad;
    private double backRightDriveAbsoluteEncoderOffsetRad;

    private boolean zeroEncoders;
    
    File file;
    PrintWriter out;
    Scanner reader;
    
    private ArrayList<SwerveModuleState> moduleStates = new ArrayList<>();
    private final SwerveModule frontLeft = new SwerveModule(
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
    public SwerveSubsystem() throws IOException{
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start(); 
        moduleStates.add(new SwerveModuleState());
        moduleStates.add(new SwerveModuleState());
        moduleStates.add(new SwerveModuleState());
        moduleStates.add(new SwerveModuleState());

        file = new File("home/lvuser/EncoderOffsets.txt");
        if(!file.exists()){
            file.createNewFile();
        }
        
        reader = new Scanner(file);
        out = new PrintWriter(file);

        frontLeftDriveAbsoluteEncoderOffsetRad = Double.parseDouble(reader.next());
        frontRightDriveAbsoluteEncoderOFfsetRad = Double.parseDouble(reader.next());
        backLeftDriveAbsoluteEncoderOffsetRad = Double.parseDouble(reader.next());
        backRightDriveAbsoluteEncoderOffsetRad = Double.parseDouble(reader.next());
    }

    public double getHeading(){
    return Math.IEEEremainder(-(gyro.getAngle()), 360);
}

public Rotation2d getRotation2d(){

    return Rotation2d.fromDegrees(getHeading());
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

    SmartDashboard.putNumber("FL Angle", frontLeft.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("FL Turning Encoder", frontLeft.getTurningPosition());
    SmartDashboard.putNumber("FR Angle", frontRight.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("BL Angle", backLeft.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("BR Angle", backRight.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("BL Left Encoder Voltage", backLeft.getAbsoluteEncoder().getVoltage());
    SmartDashboard.putNumber("5V RobotController", RobotController.getCurrent5V());
    
    SmartDashboard.putNumber("FL Target Angle", moduleStates.get(0).angle.getRadians());
    SmartDashboard.putNumber("Gyro", gyro.getAngle());

    SmartDashboard.putNumber("Mystery", getHeading());


    
    
}

public void initSendable(SendableBuilder builder, boolean zeroEncoders){
    builder.addDoubleProperty("Back Left Absolute Encoder Radians", () -> this.backLeftDriveAbsoluteEncoderOffsetRad, value -> this.backLeftDriveAbsoluteEncoderOffsetRad = value);
    builder.addDoubleProperty("Back Right Absolute Encoder Radians", () -> this.backRightDriveAbsoluteEncoderOffsetRad, value -> this.backRightDriveAbsoluteEncoderOffsetRad = value);
    builder.addDoubleProperty("Front Left Absolute Encoder Radians", () -> this.frontLeftDriveAbsoluteEncoderOffsetRad, value -> this.frontLeftDriveAbsoluteEncoderOffsetRad = value);
    builder.addDoubleProperty("Front Right Absolute Encoder Radians", () -> this.frontRightDriveAbsoluteEncoderOFfsetRad, value -> this.frontRightDriveAbsoluteEncoderOFfsetRad = value);

    builder.addBooleanProperty("Zero Encoders", () -> this.zeroEncoders, arg0 -> {
        try {
            zeroEncoders(arg0);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    });

}

public void zeroEncoders(boolean resetFlag) throws IOException{
    if (resetFlag){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
        backLeft.resetEncoders();

        out.println(String.valueOf(frontLeftDriveAbsoluteEncoderOffsetRad));
        out.println(String.valueOf(frontRightDriveAbsoluteEncoderOFfsetRad));
        out.println(String.valueOf(backLeftDriveAbsoluteEncoderOffsetRad));
        out.println(String.valueOf(backRightDriveAbsoluteEncoderOffsetRad));
    }

    
   zeroEncoders = false; 
}

}


