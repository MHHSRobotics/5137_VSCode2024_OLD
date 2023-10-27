package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;


/*This is an example auto subsystem, created by 5137, that can be used with PathPlanner (an frc community project for auto)*/
public class Auto {

    private Swerve swerve;
    private HolonomicPathFollowerConfig holonomicPathFollowerConfig;
    private ReplanningConfig replanningConfig;
    
    
    public Auto(Swerve swerve)
    {
        /*The swerve subsystem which is initialized for use by the auto system */
        this.swerve = swerve;
      
        replanningConfig = new ReplanningConfig(false, true);

        holonomicPathFollowerConfig = 
        new HolonomicPathFollowerConfig(
        Constants.SwerveConstants.drivePIDConstants, 
        Constants.SwerveConstants.anglePIDConstants, 
        Constants.AutoConstants.maxSpeed, 
        Constants.SwerveConstants.driveRadius, 
        replanningConfig);
        
        AutoBuilder.configureHolonomic(
        swerve::getPose,
        swerve::resetOdometry,
        swerve::getChassisSpeeds,
        swerve::setChassisSpeeds,
        holonomicPathFollowerConfig, 
        swerve);
      
        NamedCommands.registerCommand("exampleCommand", new InstantCommand());
    }

   
    public Command getAuto()
    {
        return new PathPlannerAuto("exampleAuto"); 
    }
  
}
