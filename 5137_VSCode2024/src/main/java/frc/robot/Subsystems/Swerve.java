package frc.robot.Subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;;

/* System created by FRFC 364. Modified and documented by FRC 5137 */
public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator; 
    public SwerveModule[] swerveMods;
    public AHRS gyro;
    public ADXRS450_GyroSim gyroSim;
    private double simYaw;

    /**
     * Swerve Substsemm that controls all modules simulataneously.
     * Contains Gyro control and simulation.
     */
    public Swerve() {
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.calibrate();
        zeroGyro();
        SimDevice gyroSim = SimDevice.create("navX-Sensor[0]", SPI.Port.kMXP.value);

        swerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        Timer.delay(1.0);
        resetModulesToAbsolute();
        
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());
        
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
        
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrivePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.zeroYaw();

    }

    public Rotation2d getYaw() {
        return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : swerveMods){
            mod.resetToAbsolute();
        }
    }
    
    @Override
    public void periodic(){
        swerveDrivePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());

        for(SwerveModule mod : swerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    @Override
    public void simulationPeriodic() {
        ChassisSpeeds chassisSpeed = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
        simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
        Unmanaged.feedEnable(20);
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(-Units.radiansToDegrees(simYaw));
    }

    
}