package frc.robot;

import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants.OIConstants;

public class RobotContainer {

    private final SwerveDriveSubsystem swerveSubsystem = new SwerveDriveSubsystem();

    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);

    public RobotContainer() {

        swerveSubsystem.setDefaultCommand(new SwerveCommand(
            swerveSubsystem,
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();

    };

    private void configureButtonBindings() {
        new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());

    }   







    /*public double getJoystickRawAxis(int id) {
        return -m_joy.getRawAxis(id);
    };*/
}
