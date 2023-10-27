package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.Subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve swerve;    
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier angleSupplier;
    private BooleanSupplier robotCentricSupplier;

    public TeleopSwerve(Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier angleSupplier, BooleanSupplier robotCentricSupplier) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.angleSupplier = angleSupplier;
        this.robotCentricSupplier = robotCentricSupplier;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double xValue = MathUtil.applyDeadband(xSupplier.getAsDouble(), Constants.stickDeadband);
        double yValue = MathUtil.applyDeadband(ySupplier.getAsDouble(), Constants.stickDeadband);
        double angleValue = MathUtil.applyDeadband(angleSupplier.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        swerve.drive(
            new Translation2d(xValue, yValue).times(Constants.SwerveConstants.maxSpeed), 
            angleValue * Constants.SwerveConstants.maxAngularVelocity, 
            !robotCentricSupplier.getAsBoolean(), 
            true
        );
}
}