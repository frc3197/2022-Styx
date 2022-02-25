package frc.robot.commands.Continuous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.DriveSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase implements Loggable {
    private final DriveSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    @Log
    private double inputX;

    private final DoubleSupplier m_translationYSupplier;
    @Log
    private double inputY;

    private final DoubleSupplier m_rotationSupplier;
    @Log
    private double inputRot;
    @Log(tabName = "Robot States")
    private String RobotDriveTrainState = "";

    private boolean fieldRelative;
    private boolean brakeMode;

    public DriveCommand(DriveSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        fieldRelative = DriveSubsystem.getFieldRelative();
        brakeMode = DriveSubsystem.getBrakeMode();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        brakeMode = DriveSubsystem.getBrakeMode();
        fieldRelative = true;
        // Stored in a new object for Oblog functionality
        inputX = m_translationXSupplier.getAsDouble();
        inputY = m_translationYSupplier.getAsDouble();
        inputRot = m_rotationSupplier.getAsDouble();

        if (!brakeMode) {
            RobotDriveTrainState = "Brake Mode Off";
            m_drivetrainSubsystem.drive(fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(inputX, inputY, inputRot,
                            m_drivetrainSubsystem.getGyroscopeRotation())
                    : new ChassisSpeeds(inputX, inputY, inputRot));
        } else {
            if (inputX != 0.0 || inputY != 0.0 || inputRot != 0.0) {
                RobotDriveTrainState = "Brake Mode On - Driving";
                m_drivetrainSubsystem.drive(fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(inputX, inputY, inputRot,
                                m_drivetrainSubsystem.getGyroscopeRotation())
                        : new ChassisSpeeds(inputX, inputY, inputRot));
            } else {
                RobotDriveTrainState = "Brake Mode On - Standby";
                m_drivetrainSubsystem.defense();
            }
        }
    }

    /**
     * @param interrupted
     */
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
