// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Toggles;

<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HoodSubsystem;

public class ToggleManualHood extends CommandBase {
  HoodSubsystem m_hoodSubsystem;

  /** Creates a new ToggleManualHood. */
  public ToggleManualHood(HoodSubsystem m_hoodSubsystem) {
    this.m_hoodSubsystem = m_hoodSubsystem;
    addRequirements(m_hoodSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hoodSubsystem.setHood(RobotContainer.getDriver2().getYLeft(.1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hoodSubsystem.setHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
