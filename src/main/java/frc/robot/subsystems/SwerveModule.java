// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;



public class SwerveModule extends SubsystemBase {

  private final CANSparkMax driverMotor;
  private final CANSparkMax spinMotor;

  private final RelativeEncoder driverEncoder;
  private final RelativeEncoder spinEncoder;

  private final AnalogInput absoluteEncoder;
  private final double absoluteEncoderOffsetRadians;
  private final boolean absoluteEncoderInversed;


  /** Creates a new SwerveModule. */
  public SwerveModule(int driverMotorPort, int spinMotorPort, boolean driveMotorInverse, boolean spinMotorInverse, int absoluteEncoderChannel, double absoluteEncoderOffset, boolean absoluteEncoderInversed) {

    driverMotor = new CANSparkMax(driverMotorPort, MotorType.kBrushless);
    spinMotor = new CANSparkMax(spinMotorPort, MotorType.kBrushless);

    //driverMotor.setInverted(driveMotorInverse);
    //spinMotor.setInverted(spinMotorInverse);

    driverEncoder = driverMotor.getEncoder();
    spinEncoder = spinMotor.getEncoder();

    this.absoluteEncoderOffsetRadians = absoluteEncoderOffset;
    this.absoluteEncoderInversed = absoluteEncoderInversed;
    absoluteEncoder = new AnalogInput(absoluteEncoderChannel);

    //NEED TO ADD CONVERSIONS HERE

    resetEncoders();
  }

  public double getDrivePosition() {
    return driverEncoder.getPosition();
  }

  public double getSpinPosition() {
    return spinEncoder.getPosition();
  }

  public double getDriverVelocity() {
    return driverEncoder.getVelocity();
  }

  public double getSpinVelocity() {
    return spinEncoder.getVelocity();
  }

  public void resetEncoders() {
    driverEncoder.setPosition(0);
    spinEncoder.setPosition(0);
  }

  /**public SwerveModuleState getState() {
    return SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }**/


  public double getAbsoluteEncoderRadians() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getCurrent5V();
    angle *= 2 * Math.PI;
    angle -= absoluteEncoderOffsetRadians;
    if (absoluteEncoderInversed){
      return angle * -1;
    } else {
      return angle;
    }
  }



  public void stop() {
    driverMotor.set(0);
    spinMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
