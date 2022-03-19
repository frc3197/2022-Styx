// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//*
package frc.robot.commands.Groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Actions.General.Lifter.FeedToUpper;
import frc.robot.commands.Actions.General.Lifter.ShootUpper;
import frc.robot.other.Wait;
import frc.robot.subsystems.Shooter.LifterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSequence extends SequentialCommandGroup {
  LifterSubsystem m_lifterSubsystem;
  // Creates a new shootSequence. 
  public ShootSequence( LifterSubsystem m_lifterSubsystem){
    
    this.m_lifterSubsystem = m_lifterSubsystem;
    addRequirements();
    addCommands(new ShootUpper(m_lifterSubsystem),new FeedToUpper(m_lifterSubsystem),new ShootUpper(m_lifterSubsystem));
  }
}
