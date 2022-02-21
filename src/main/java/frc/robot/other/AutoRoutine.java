// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber.ClimberArm;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Intake.IntakeArm;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.HoodSubsystem;
import frc.robot.subsystems.Shooter.LifterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/** Add your docs here. */
public class AutoRoutine extends SequentialCommandGroup {
  
    private static DriveSubsystem m_driveSubsystem = RobotContainer.getDriveSubsystem();
    private static ClimberSubsystem m_climberSubsystem = RobotContainer.getClimberSubsystem();
    private static ClimberArm m_climberArmSubsystem = RobotContainer.getClimberArmSubsystem();
    private static HoodSubsystem m_hoodSubsystem = RobotContainer.getHoodSubsystem();
    private static IntakeSubsystem m_intakeSubsystem = RobotContainer.getIntakeSubsystem();
    private static IntakeArm m_intakeArmSubsystem = RobotContainer.getIntakeArmSubsystem();
    private static LifterSubsystem m_lifterSubsystem = RobotContainer.getLifterSubsystem();
    private static ShooterSubsystem m_shooterSubsystem = RobotContainer.getShooterSubsystem();
    

    public AutoRoutine(Command... commands){
        addCommands(commands);
        //TODO: Check if needed / move to before addCommands(); 
        addRequirements(m_intakeArmSubsystem);
        addRequirements(m_driveSubsystem);
        addRequirements(m_climberSubsystem);
        addRequirements(m_climberArmSubsystem);
        addRequirements(m_hoodSubsystem);
        addRequirements(m_intakeSubsystem);
        addRequirements(m_lifterSubsystem);
        addRequirements(m_shooterSubsystem); 
        
    }

    public IntakeArm getIntakeArmSubsystem(){
      return m_intakeArmSubsystem;
    }
    public DriveSubsystem getDriveSubsystem() {
        return m_driveSubsystem;
      }
      
      public ClimberArm getClimberArmSubsystem(){
        return m_climberArmSubsystem;
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
