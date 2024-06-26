// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Intake extends Command {
  private Shooter m_shooter;


  /** Creates a new Intake. */
  public Intake(Shooter shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.intakeForAutonomous(); // startIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

if(m_shooter.hasNote()){
 // m_shooter.stop();
 this.end(false);

}else {
  m_shooter.startIntake();
}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_shooter.stop();
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return m_shooter.hasNote();
  }
}
