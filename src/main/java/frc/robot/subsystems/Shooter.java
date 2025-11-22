// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;
import static frc.robot.constants.Constants.Shooter.*;

public class Shooter extends SubsystemBase {
  private TalonFX m_Shooter;
  final MotionMagicVelocityDutyCycle m_Request = new MotionMagicVelocityDutyCycle(0);
  /** Creates a new shooter. */
  public Shooter() 
  {
    m_Shooter = new TalonFX(IDConstants.shooterID);
    

    // in init function
    var shooterConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    shooterConfigs.Slot0.kS = slot0S; 
    shooterConfigs.Slot0.kV = slot0V; 
    shooterConfigs.Slot0.kA = slot0A; 
    shooterConfigs.Slot0.kP = slot0P;
    shooterConfigs.Slot0.kI = slot0I; 
    shooterConfigs.Slot0.kD = slot0D; 

    // set Motion Magic settings
    shooterConfigs.MotionMagic.MotionMagicCruiseVelocity = velocity; 
    shooterConfigs.MotionMagic.MotionMagicAcceleration = acceleration;


    m_Shooter.getConfigurator().apply(shooterConfigs);
  }

  public Command setSpeedCommand(double speed)
  {
    return this.run(() -> m_Shooter.setControl(m_Request.withVelocity(speed)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
