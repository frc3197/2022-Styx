// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** Add your docs here. */
public class AutoRoutine extends SequentialCommandGroup {
  
    private static DriveSubsystem m_driveSubsystem = RobotContainer.getDriveSubsystem();
    private static ClimberSubsystem m_climberSubsystem = null;
    private static HoodSubsystem m_hoodSubsystem = RobotContainer.getHoodSubsystem();
    private static IntakeSubsystem m_intakeSubsystem = RobotContainer.getIntakeSubsystem();
    private static LifterSubsystem m_lifterSubsystem = RobotContainer.getLifterSubsystem();
    private static ShooterSubsystem m_shooterSubsystem = RobotContainer.getShooterSubsystem();
    
    public AutoRoutine(Command... commands){
        addCommands(commands);
        //TODO: Check if needed / move to before addCommands(); 
        
        addRequirements(m_driveSubsystem);
        addRequirements(m_climberSubsystem);
        addRequirements(m_hoodSubsystem);
        addRequirements(m_intakeSubsystem);
        addRequirements(m_lifterSubsystem);
        addRequirements(m_shooterSubsystem); 
        
    }


    public DriveSubsystem getDriveSubsystem() {
        return m_driveSubsystem;
      }
    
      public ClimberSubsystem getClimberSubsystem() {
        return m_climberSubsystem;
      }
    
      public HoodSubsystem getHoodSubsystem() {
        return m_hoodSubsystem;
      }
    
      public IntakeSubsystem getIntakeSubsystem() {
        return m_intakeSubsystem;
      }
    
      public LifterSubsystem getLifterSubsystem() {
        return m_lifterSubsystem;
      }
    
      public ShooterSubsystem getShooterSubsystem() {
        return m_shooterSubsystem;
      }



}
