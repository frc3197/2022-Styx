// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.other;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.filters.Filter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.other.extra_libraries.FilteredController;

public class RumbleForTime extends CommandBase {
  FilteredController xboxController;
  double time;
  Timer timer = new Timer();

  /** Creates a new RumbleForTime. */
  public RumbleForTime(FilteredController xboxController, double time) {
    this.xboxController = xboxController;
    this.time = time;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Rumble Data", "is Rumbling");
    xboxController.setRumble(.2);
    xboxController.setRumble(.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    xboxController.setRumble(0);
    xboxController.setRumble(0);
    SmartDashboard.putString("Rumble Data", "isnt Rumbling");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
