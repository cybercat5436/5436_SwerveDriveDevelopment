package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;;

public class SwerveSubsystem extends SubsystemBase {
    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    public final SwerveModule frontLeft = new SwerveModule(
        Constants.kFrontLeftDriveMotorPort,
        Constants.kFrontLeftTurningMotorPort,
        Constants.kFrontLeftDriveEncoderReversed,
        Constants.kFrontLeftTurningEncoderReversed,
        Constants.kFrontLeftDriveAbsoluteEncoderPort,
        Constants.kFrontLeftAbsoluteEncoderOffsetRad,
        Constants.kFrontLeftDriveAbsoluteEncoderReversed);

    public final SwerveModule frontRight = new SwerveModule(
        Constants.kFrontRightDriveMotorPort,
        Constants.kFrontRightTurningMotorPort,
        Constants.kFrontRightDriveEncoderReversed,
        Constants.kFrontRightTurningEncoderReversed,
        Constants.kFrontRightDriveAbsoluteEncoderPort,
        Constants.kFrontRightAbsoluteEncoderOffsetRad,
        Constants.kFrontRightDriveAbsoluteEncoderReversed);

    public final SwerveModule backLeft = new SwerveModule(
        Constants.kBackLeftDriveMotorPort,
        Constants.kBackLeftTurningMotorPort,
        Constants.kBackLeftDriveEncoderReversed,
        Constants.kBackLeftTurningEncoderReversed,
        Constants.kBackLeftDriveAbsoluteEncoderPort,
        Constants.kBackLeftAbsoluteEncoderOffsetRad,
        Constants.kBackLeftDriveAbsoluteEncoderReversed);

    public final SwerveModule backRight = new SwerveModule(
        Constants.kBackRightDriveMotorPort,
        Constants.kBackRightTurningMotorPort,
        Constants.kBackRightDriveEncoderReversed,
        Constants.kBackRightTurningEncoderReversed,
        Constants.kBackRightDriveAbsoluteEncoderPort,
        Constants.kBackRightAbsoluteEncoderOffsetRad,
        Constants.kBackRightDriveAbsoluteEncoderReversed);

        public SwerveSubsystem() {
            new Thread(() -> {
                try {
                    Thread.sleep(1000);
                } catch (Exception e){
    
                }

            }).start();

        }
public void zeroHeading() { 
    gyro.reset();
}

public double getHeading(){
   return Math.IEEEremainder(gyro.getAngle(), 360); 
}

public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
}

@Override
public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading());
}

public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}

public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);    
}
}