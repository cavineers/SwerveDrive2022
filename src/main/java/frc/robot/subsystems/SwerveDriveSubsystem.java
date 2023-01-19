package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI;


public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveCanID, DriveConstants.kFrontLeftTurningCanID, DriveConstants.kFrontLeftDriveEncoderReversed, DriveConstants.kFrontLeftTurningEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(DriveConstants.kFrontRightDriveCanID, DriveConstants.kFrontRightTurningCanID, DriveConstants.kFrontRightDriveEncoderReversed, DriveConstants.kFrontRightTurningEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(DriveConstants.kBackLeftDriveCanID, DriveConstants.kBackLeftTurningCanID, DriveConstants.kBackLeftDriveEncoderReversed, DriveConstants.kBackLeftTurningEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(DriveConstants.kBackRightDriveCanID, DriveConstants.kBackRightTurningCanID, DriveConstants.kBackRightDriveEncoderReversed, DriveConstants.kBackRightTurningEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        new Translation2d(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0),
        new Translation2d(DriveConstants.kTrackWidth / 2.0, -DriveConstants.kWheelBase / 2.0),
        new Translation2d(-DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0),
        new Translation2d(-DriveConstants.kTrackWidth / 2.0, -DriveConstants.kWheelBase / 2.0)
    );

    public double getHeading(){
        return Math.IEEEremainder(0, 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getRotation2d());
    SwerveDriveOdometry m_odometer = m_odometry;

    public SwerveDriveSubsystem() {
        zeroHeading();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Pose2d getPose() {
        return m_odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometer.resetPosition(pose, getRotation2d());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void periodic(){
        m_odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
        backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
        rotation *= 2.0 / Math.hypot(DriveConstants.kWheelBase, DriveConstants.kTrackWidth);
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                    Rotation2d.fromDegrees(getHeading()));
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }
}