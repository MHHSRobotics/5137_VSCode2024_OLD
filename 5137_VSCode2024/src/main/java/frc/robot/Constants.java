package frc.robot;


import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final double stickDeadband = 0.1;

    public static final class SwerveConstants {
        
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final double voltageComp = 12.0;
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.75);
        public static final double driveRadius = Units.inchesToMeters(21.0);
        public static final double wheelBase = Units.inchesToMeters(24.75); 
        public static final double wheelCircumference = Units.inchesToMeters(4.0);
    

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = (6.75 / 1.0); // SDS MK4i L2
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // SDS Mk4i

        /* Motor Inverts */
        public static final boolean angleMotorInvert = true;
        public static final boolean driveMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 30;
        public static final int anglePeakCurrentLimit = 45;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 40;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.3;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;
        public static final PIDConstants anglePIDConstants = new PIDConstants(angleKP, angleKI, angleKD);

        /* Drive Motor PID Values */
        public static final double driveKP = 1.6021; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.02391;
        public static final double driveKF = 0.0;
        public static final PIDConstants drivePIDConstants = new PIDConstants(driveKP, driveKI, driveKD);
        
        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.79074 / 12); 
        public static final double driveKV = (.194587 / 12);
        public static final double driveKA = (0.010932 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;
        
        /* Gear Ratio Stuff */
        public static final double angleRotationsToRadians = 360.0 /angleGearRatio;
        public static final double driveRotationsToMeters = wheelCircumference/driveGearRatio;
        public static final double driveRPMToMetersPerSecond = driveRotationsToMeters/60.0;
         

        public static final Translation2d[] kModuleTranslations = {
            new Translation2d(wheelBase / 2, trackWidth / 2),
            new Translation2d(wheelBase / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2, trackWidth / 2),
            new Translation2d(-wheelBase / 2, -trackWidth / 2)
        };

        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT
          };
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(138.779);
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset); 
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(142.031);
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(201.797);
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(312.979);
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double maxSpeed = SwerveConstants.maxSpeed;
        public static final double maxAcceleration = 3;
        public static final double maxAngularSpeed = SwerveConstants.maxAngularVelocity;
        public static final double maxAngularAcceleration = Math.PI;
    
        public static final double kPXController = 1; 
        public static final double kPYController = 1;
        public static final double kPAngleController = 1;
    
        
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints angleControllerConstraints =
            new TrapezoidProfile.Constraints(
                maxAngularSpeed, maxAngularAcceleration);

            };


}
  

