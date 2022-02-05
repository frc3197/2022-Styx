// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.commands.Actions.General.Shoot;
import frc.robot.commands.Continuous.Spool;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSequence extends ParallelRaceGroup {
  ShooterSubsystem m_shooterSubsystem;
  LifterSubsystem m_lifterSubsystem;
  /** Creates a new shootSequence. */
  public ShootSequence(ShooterSubsystem m_shooterSubsystem, LifterSubsystem m_lifterSubsystem){
    this.m_shooterSubsystem = m_shooterSubsystem;
    this.m_lifterSubsystem = m_lifterSubsystem;
    addRequirements();
    addCommands(new Spool(m_shooterSubsystem, Constants.subsystems.shooter.targetRPM), new Shoot(m_lifterSubsystem));
  }
}
