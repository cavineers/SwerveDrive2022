package frc.robot;

import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BalanceControlCommand;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.SwitchFunction;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.BalanceControlCommand;


public class RobotContainer {

   
    private final SwerveDriveSubsystem swerveSubsystem = new SwerveDriveSubsystem();
    public Command m_balance;
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
    private JoystickButton buttonX = new JoystickButton(driverJoystick, 3);
    private JoystickButton buttonY = new JoystickButton(driverJoystick, 4);

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
        buttonX.onTrue(new InstantCommand(){
            public void initialize() {
                System.out.println("Button X pressed");
                m_balance = new BalanceControlCommand(swerveSubsystem);
                m_balance.schedule();
            }
        }
        ); 

    }
}
