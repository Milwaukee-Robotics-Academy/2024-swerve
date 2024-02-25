// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The elevator subsystem uses PID to go to a given height. Unfortunately, in it's current state PID
 * values for simulation are different than in the real world do to minor differences.
 */
public class Shooter extends SubsystemBase 
{
    private CANSparkMax m_flywheel = new CANSparkMax(ShooterConstants.kFlywheelMotorPort, MotorType.kBrushless);
    private RelativeEncoder m_flywheelEncoder = m_flywheel.getEncoder();
    private CANSparkMax m_triggerMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);
    private RelativeEncoder m_triggerEncoder = m_flywheel.getEncoder();
    private CANSparkMax m_flywheelLeft = new CANSparkMax(ShooterConstants.kFlywheelLeftMotorPort, MotorType.kBrushless);
     private RelativeEncoder m_flywheelEncoderLeft = m_flywheel.getEncoder();
    //  private CANSparkMax m_triggerMotorLeft = new CANSparkMax(ShooterConstants.kShooterLeftMotorPort, MotorType.kBrushless);
    //  private RelativeEncoder m_triggerEncoderLeft = m_flywheel.getEncoder();
    public Shooter()
    {
      m_flywheel.restoreFactoryDefaults();
      m_flywheel.setSmartCurrentLimit(80);
      m_flywheel.setIdleMode(IdleMode.kBrake);
      m_flywheel.setInverted(false);
      m_flywheelLeft.restoreFactoryDefaults();
       m_flywheelLeft.setSmartCurrentLimit(80);
       m_flywheelLeft.setIdleMode(IdleMode.kBrake);
       m_flywheelLeft.setInverted(true);
      m_triggerMotor.restoreFactoryDefaults();
      m_triggerMotor.setSmartCurrentLimit(80);
      m_triggerMotor.setIdleMode(IdleMode.kBrake);
      m_triggerMotor.setInverted(false);
      //  m_triggerMotorLeft.restoreFactoryDefaults();
      //  m_triggerMotorLeft.setSmartCurrentLimit(80);
      //  m_triggerMotorLeft.setIdleMode(IdleMode.kBrake);
      //  m_triggerMotorLeft.setInverted(true);
      SmartDashboard.putNumber("FW-Encoder/speed",m_flywheelEncoder.getVelocity());
      SmartDashboard.putNumber("FW-Encoder/distance",m_flywheelEncoder.getPosition());
      SmartDashboard.putNumber("Trigger-Encoder/speed",m_flywheelEncoder.getVelocity());
      SmartDashboard.putNumber("Trigger-Encoder/distance",m_flywheelEncoder.getPosition());


      this.stop();
    }

  public void set(Double speed)
  {
    m_triggerMotor.set(speed);
    m_flywheel.set(speed);
  }
  public double getTopMotorSpeed() {
    return m_flywheel.getEncoder().getVelocity();
}

public double getBottomMotorSpeed() {
    return m_triggerMotor.getEncoder().getVelocity();
}

  public void stop()
  {
    m_triggerMotor.stopMotor();
    m_flywheel.stopMotor();
      // m_triggerMotorLeft.stopMotor();
     m_flywheelLeft.stopMotor();
  }


  public void intake()
  {
    m_triggerMotor.set(-0.25);
    m_flywheel.set(-0.25);
    // m_triggerMotorLeft.set(-0.25);
     m_flywheelLeft.set(-0.25);
  }

  public void readyFlywheel()
  {
    m_flywheel.set(1);
     m_flywheelLeft.set(1);
  }

  public void shoot()
  {
    m_triggerMotor.set(1);
    // m_triggerMotorLeft.set(.25);
    m_flywheel.set(1);
    m_flywheelLeft.set(1);
  }
  
  public void setMotorSpeed(double topSpeed, double bottomSpeed) {
    m_flywheel.set(topSpeed);
    m_triggerMotor.set(bottomSpeed);
     m_flywheelLeft.set(topSpeed);
    //  m_triggerMotorLeft.set(bottomSpeed);
}
  /** The log method puts interesting information to the SmartDashboard. */
  public void log() 
  {
  //  SmartDashboard.putData("Shooter Speed", m_pot);
  }


  /** Call log method every loop. */
  @Override
  public void periodic() 
  {
    log();
    SmartDashboard.putNumber("FlywheelMotorSpeed", getTopMotorSpeed());
    SmartDashboard.putNumber("TriggerMotorSpeed", getBottomMotorSpeed());
  }
}
