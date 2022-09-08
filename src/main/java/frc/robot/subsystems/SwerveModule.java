// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class SwerveModule{

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final PIDController turningPidController;

  private final AnalogInput absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;


  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
   int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, IdleMode driveMode, IdleMode turningMode) {

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new AnalogInput(absoluteEncoderId);

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    driveMotor.clearFaults();
    turningMotor.clearFaults();

    driveMotor.restoreFactoryDefaults();
    turningMotor.restoreFactoryDefaults();

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveMotor.setIdleMode(driveMode);
    turningMotor.setIdleMode(turningMode);

    

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    //NEED TO ADD CONVERSIONS HERE

    resetEncoders();
  }

  /*
  public double getDrivePosition() {
    return driverEncoder.getPosition();
  }

  public double getSpinPosition() {
    return spinEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driverEncoder.getVelocity();
  }

  public double getSpinVelocity() {
    return spinEncoder.getVelocity();
  }

  public void resetEncoders() {
    driverEncoder.setPosition(0);
    spinEncoder.setPosition(0);
  }
  */

  private void resetEncoders() {
    driveEncoder.setPosition(0.0);
    System.out.println(String.format("turningEncoder %.2f", turningEncoder.getPosition()));
    System.out.println(String.format("absoluteEncoder %.2f", this.getAbsoluteEncoderRadians()));
    turningEncoder.setPosition(getAbsoluteEncoderRadians());
    System.out.println(String.format("after zero turningEncoder %.2f", turningEncoder.getPosition()));
    System.out.println(String.format("after zero absoluteEncoder %.2f", this.getAbsoluteEncoderRadians()));
  }

  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  public double getTurningPosition(){
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public AnalogInput getAbsoluteEncoder(){
    return this.absoluteEncoder;
  }



  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  //definetly ot right but gets rid of error
  // private SwerveModuleState SwerveModuleState(double driveVelocity, Rotation2d rotation2d) {
  //   return null;
  // }

  public void setDesiredState(SwerveModuleState state){
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }      
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(turningPidController.calculate(turningEncoder.getPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    double driveMotorPower = state.speedMetersPerSecond /DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    SmartDashboard.putNumber("Drive Motor Power", driveMotorPower);

  }


  public double getAbsoluteEncoderRadians() {
    // double angle = absoluteEncoder.getVoltage() / RobotController.getCurrent5V();
    double angle = absoluteEncoder.getVoltage() / 5.0;
    angle = (Math.PI * 2) * angle - Math.PI;
    if (absoluteEncoderReversed){
      angle *= -1;
    } 
    angle -= absoluteEncoderOffsetRad;
    angle = boundAngle(angle);
    return angle;
  }

  private double boundAngle(double inputAngleRad){
    double outputAngleRad = inputAngleRad;
    if (outputAngleRad <= -Math.PI){
      outputAngleRad += (2*Math.PI);
    } else if(outputAngleRad > (Math.PI)){
      outputAngleRad -= (2 * Math.PI);
    }
    return outputAngleRad;

  }



  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }



  /**@Override
  public void periodic() {
   This method will be called once per scheduler run
  } **/
}
