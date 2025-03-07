// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Configs;

public class MAXSwerveModule{
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final CANcoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARK MAX, and an external CANCoder.
   *
   * @param drivingCANId       CAN ID for the driving motor controller.
   * @param turningCANId       CAN ID for the turning motor controller.
   * @param turningEncoderCANId CAN ID for the external CANCoder used for turning.
   * @param chassisAngularOffset Offset to align the module's chassis angle.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, int turningEncoderCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    // Instead of using the REV absolute encoder, create a new CANCoder instance.
    m_turningEncoder = new CANcoder(turningEncoderCANId);

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Configure the external CANCoder.
    CANcoderConfiguration caNcoderConfiguration = new CANcoderConfiguration();
    // You can adjust the configuration as needed (for example, sensor direction or range).
    m_turningEncoder.getConfigurator(CANcoderConfiguration);

    // Apply the respective configurations to the SPARKS.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    // Initialize the desired state angle using the CANCoder reading (converted to radians).
    m_desiredState.angle = new Rotation2d((Angle) m_turningEncoder.getAbsolutePosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Convert the CANCoder's absolute position from degrees to radians,
    // then apply the chassis angular offset.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d((Angle) m_turningEncoder.getAbsolutePosition()) - m_chassisAngularOffset());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition()) - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than necessary.
    correctedDesiredState.optimize(new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition())));

    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

}