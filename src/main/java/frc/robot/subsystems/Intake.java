// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Intake extends SubsystemBase {
   private CANSparkMax m_leftIntake = new CANSparkMax(IntakeConstants.kLeftMotorPort, MotorType.kBrushless);
    private CANSparkMax m_rightIntake = new CANSparkMax(IntakeConstants.kRightMotorPort, MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {

      m_leftIntake.restoreFactoryDefaults();
      m_leftIntake.setSmartCurrentLimit(20);
      m_leftIntake.setIdleMode(IdleMode.kBrake);
      m_leftIntake.setInverted(true);
      m_rightIntake.restoreFactoryDefaults();
       m_rightIntake.setSmartCurrentLimit(20);
       m_rightIntake.setIdleMode(IdleMode.kBrake);
       m_rightIntake.setInverted(true);
     

  }

  public void in(){
    m_leftIntake.set(.75);
    m_rightIntake.set(.75);
  }

  public void out(){
    m_leftIntake.set(-.5);
    m_rightIntake.set(-.5);
  }

  public void stop(){
    m_leftIntake.set(0);
    m_rightIntake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
