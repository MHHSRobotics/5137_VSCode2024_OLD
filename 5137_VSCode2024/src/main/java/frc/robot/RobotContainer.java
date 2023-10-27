package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.*;

public class RobotContainer {
    

    private final Swerve swerve = new Swerve();
    private final Joystick driver = new Joystick(0);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final FieldSim fieldSim = new FieldSim(swerve);


    public RobotContainer() {
       
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driver.getRawAxis(1),
                () -> -driver.getRawAxis(0),
                () -> -driver.getRawAxis(4),
                () -> robotCentric.getAsBoolean()
            ));

        fieldSim.initSim();
        configureButtonBindings();
    }

    private void configureButtonBindings() {
    
        new JoystickButton(driver, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    }

    public Command getAutoCommand() {
        return new InstantCommand();
    }
    
    public void periodic() {
      }
}