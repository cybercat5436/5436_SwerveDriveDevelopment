// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

/** Add your docs here. */
public class SwerveSubsystem extends SubsystemBase{


    private final SwerveModule frontLeft = new SwerveModule();
    private final SwerveModule frontRight = new SwerveModule();
    private final SwerveModule backLeft = new SwerveModule();
    private final SwerveModule backRight = new SwerveModule();

    private final AHRS gyro = new AHRS(0);

    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
}
