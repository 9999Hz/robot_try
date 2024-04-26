// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final CANSparkMax turnIntakeMotor;

  private final CANcoder turnIntakeCancoder;
  private final CANcoderConfiguration turnIntakeMotorCaNcoderConfiguration;

  private final PIDController turnIntakePidController;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
    turnIntakeMotor = new CANSparkMax(0, MotorType.kBrushless);

    turnIntakePidController = new PIDController(IntakeConstants.kp, IntakeConstants.ki, IntakeConstants.kd);

    intakeMotor.restoreFactoryDefaults();
    turnIntakeMotor.restoreFactoryDefaults();

    intakeMotor.setInverted(false);
    turnIntakeMotor.setInverted(false);

    intakeMotor.setIdleMode(IdleMode.kCoast);
    turnIntakeMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.burnFlash();
    turnIntakeMotor.burnFlash();


    turnIntakeCancoder = new CANcoder(0);
    turnIntakeMotorCaNcoderConfiguration = new CANcoderConfiguration();

    turnIntakeMotorCaNcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    turnIntakeMotorCaNcoderConfiguration.MagnetSensor.MagnetOffset = Constants.IntakeConstants.turnIntakeMotorCaNcoderConfigurationOffset;
    turnIntakeMotorCaNcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    turnIntakeCancoder.getConfigurator().apply(turnIntakeMotorCaNcoderConfiguration);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
