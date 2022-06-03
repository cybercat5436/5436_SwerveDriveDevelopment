package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {
    private SwerveSubsystem swerveSubsystem;
    private Supplier <Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private Supplier<Boolean> fieldOrientedFunction;
    private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.5);
    private XboxController xboxController;


    public  SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
                Supplier<Double>xSpdFunction, Supplier<Double>ySpdFunction, Supplier<Double>turningSpdFunction,
                Supplier<Boolean> fieldOrientedFunction){
                    this.swerveSubsystem = swerveSubsystem;
                    this.xSpdFunction = xSpdFunction;
                    this.ySpdFunction = ySpdFunction;
                    this. turningSpdFunction = turningSpdFunction;
                    this.fieldOrientedFunction = fieldOrientedFunction;

                }

    @Override

    public void execute(){
        // get real time joyStick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        //apply dead band 
        xSpeed = Math.abs(xSpeed) > OIConstants.K_DEADBAND ? xSpeed : 0.0 *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = Math.abs(ySpeed) > OIConstants.K_DEADBAND ? ySpeed : 0.0 *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.K_DEADBAND ? turningSpeed : 0.0;

        // make driving smoother
        xSpeed = slewRateLimiter.calculate(xSpeed) *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = slewRateLimiter.calculate(ySpeed) *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = slewRateLimiter.calculate(turningSpeed)
        *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        // convert speeds to reference frames
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()){
            //need to define in constants.java
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
                ;
            
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);

    }

    @Override
    public void end(boolean interupted){
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
