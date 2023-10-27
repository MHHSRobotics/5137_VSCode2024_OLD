package frc.robot.Subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CTREConfigs;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;


import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/* Swerve Module Subsystem that controls individual modules. Created by FRC 364 and modified by FRC 5137. */
public class SwerveModule extends SubsystemBase{
    public int moduleNumber;
    private Rotation2d angleOffset;
    public Rotation2d lastAngle;

    private double simDrivePosition;
    private double simDriveVelocity;
    private double simAngleDifference;
    private double simAngleIncrement;
    private double simAngle;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    public CANCoder angleEncoder;

    private SparkMaxPIDController driveController;
    private SparkMaxPIDController angleController;

    private CTREConfigs ctreConfigs; 

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    /**
     * Object for controlling each individual swerve module.
     * @param moduleNumber the ID assigned to each swerveModule(0-3). Check Constants for explanation of each number. 
     * @param moduleConstants the natural angle offset of the module, when facing straight, according to the absolute encoder. 
     */
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        ctreConfigs = new CTREConfigs();
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
       
       /*For simulation only */
        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(angleMotor, DCMotor.getNEO(1));
            driveController.setP(1, 2);
          }
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveMotor.set(percentOutput);
        }
        else {
            driveController.setReference(
                desiredState.speedMetersPerSecond,
                 ControlType.kVelocity, 0, 
                 feedforward.calculate(desiredState.speedMetersPerSecond));
        }
        
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
        if (RobotBase.isSimulation()) {
            simUpdateDrivePosition(desiredState);
            simAngle = angle.getDegrees();
          }
    }

    private Rotation2d getAngle(){
        
        
        if(RobotBase.isReal())
        {
            return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
        }
        else 
        {
            return Rotation2d.fromDegrees(simAngle);
        }
        
       
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        System.out.println("absolute" + absolutePosition);  
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){      
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        angleMotor.restoreFactoryDefaults();
        angleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
        angleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
        angleController.setP(Constants.SwerveConstants.angleKP);
        angleController.setI(Constants.SwerveConstants.angleKI);
        angleController.setD(Constants.SwerveConstants.angleKD);
        angleController.setFF(Constants.SwerveConstants.angleKF);
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMaxInput(360);
        angleController.setPositionPIDWrappingMinInput(1);//TODO: Check if needs Position PID Wrapping or PID Factor changes
        resetToAbsolute();
        integratedAngleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleRotationsToRadians);
        integratedAngleEncoder.setVelocityConversionFactor(Constants.SwerveConstants.angleRPMToRadiansPerSecond);
        angleMotor.burnFlash();
    }

    private void configDriveMotor(){        
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
        driveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
        driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
        driveController.setP(Constants.SwerveConstants.driveKP);
        driveController.setI(Constants.SwerveConstants.driveKI); 
        driveController.setD(Constants.SwerveConstants.driveKD);
        driveController.setFF(Constants.SwerveConstants.angleKF);
        driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveRotationsToMeters);
        driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveRPMToMetersPerSecond);
        driveEncoder.setPosition(0); 
        driveMotor.burnFlash();
    }

    public double getVelocity()
    {
        if(RobotBase.isReal())
        return driveEncoder.getVelocity();
      else
        return simDriveVelocity;
    }

    public double getDrivePosition()
    {
        if(RobotBase.isReal())
      return driveEncoder.getPosition();
    else
      return simDrivePosition;
  
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getVelocity(), getAngle()); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), getAngle());
    }
    private void simUpdateDrivePosition(SwerveModuleState state) {
        simDriveVelocity = state.speedMetersPerSecond;
        double distancePer20Ms = simDriveVelocity / 50.0;
        simDrivePosition += distancePer20Ms;
      }
      private void simTurnPosition(double angle) {
        if (angle != lastAngle.getDegrees() && simAngleIncrement == 0) {
          simAngleDifference = angle - lastAngle.getDegrees();
          simAngleIncrement = simAngleDifference / 20.0;// 10*20ms = .2 sec move time
        }
    
}

@Override
public void periodic()
{
}
@Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}