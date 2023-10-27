// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SwerveConstants.kModuleTranslations;


/*This subsystem we pulled from Team 4201, the Vitruvian bots.
 It allows us to simulate a swerve drive in the SIM GUI.
 It has been modified by 5137 and is not completely representative 
 of their individual project and format*/
public class FieldSim extends SubsystemBase{
 
  private final Swerve m_swerveDrive;
  private double m_simYaw;
  /* This field is used in the simulatiion GUI */
  private final Field2d m_field2d = new Field2d();

  /*Stores the position of each swerve module */
  private Pose2d[] m_swerveModulePoses = {
          new Pose2d(),
          new Pose2d(),
          new Pose2d(),
          new Pose2d()
  };

    /*Constructor which provides a swerve subsystem for the simulation to access */
  public FieldSim(Swerve s_Swerve) {
    m_swerveDrive = s_Swerve;
  }

  public void initSim() {}

  public Field2d getField2d() {
    return m_field2d;
  }
  /*Updates the robotpose to be used in simulation */
  private void updateRobotPoses() {
    m_field2d.setRobotPose(m_swerveDrive.getPose());

    
    for (int i = 0; i < kModuleTranslations.length; i++) {
      Translation2d updatedPositions =
              kModuleTranslations[i]
                      .rotateBy(m_swerveDrive.getPose().getRotation())
                      .plus(m_swerveDrive.getPose().getTranslation());
      m_swerveModulePoses[i] =
              new Pose2d(
                      updatedPositions,
                      new Rotation2d(m_swerveDrive.swerveMods[i].angleEncoder.getPosition())
                              .plus(m_swerveDrive.getYaw()));
    }

    m_field2d.getObject("Swerve Modules").setPoses(m_swerveModulePoses);
  }

    /*Puts data onto the SIM GUI  */
  @Override
  public void periodic() {
    updateRobotPoses();

    if (RobotBase.isSimulation()) simulationPeriodic();

    SmartDashboard.putData("Field2d", m_field2d);
  }

}
