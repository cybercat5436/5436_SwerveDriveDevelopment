package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier <Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;

    public SwerveSubsystemCmd(SwerveSubsystem swerveSubsystem,
                Supplier<Double>xSpdFunction, Supplier<Double>ySpdFunction, Supplier<Double>turningSpdFunction,
                Supplier<Boolean> fieldOrientedFunction){
                    this.swerveSubsystem = swerveSubsystem;
                    this.xSpdFunction = xSpdFunction;
                    this.ySpdFunction = ySpdFunction;
                    this. turningSpdFunction = turningSpdFunction;
                    this.fieldOrientedFunction = fieldOrientedFunction;

                    addRequirements(swerveSubsystem);

                }

    @Override

    public void execute(){
        // get real time joyStick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        //apply dead band 
        xSpeed = Math.abs(xSpeed) > OIConstanrs.kDeadBand ? xSpeed : 0.0; *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = Math.abs(ySpeed) > OIConstanrs.kDeadBand ? ySpeed : 0.0; *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = Math.abs(turningSpeed) > OIConstanrs.kDeadBand ? turningSpeed : 0.0;

        // make driving smoother
        xSpeed = xLimiter.calculate(xSpeed) *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
        *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        // convert speeds to reference frames
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()){
            //need to define in constants.java
            chassisSpeeds = ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
            
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
        return fals;
    }

}
