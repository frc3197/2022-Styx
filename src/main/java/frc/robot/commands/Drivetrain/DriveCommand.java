package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.DriveSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.DoubleSupplier;

@SuppressWarnings("unused")
public class DriveCommand extends CommandBase implements Loggable {
    private final DriveSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    @Log
    private double inputX;
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(7);
    private final DoubleSupplier m_translationYSupplier;
    @Log
    private double inputY;
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(7);

    private final DoubleSupplier m_rotationSupplier;
    @Log
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(7);
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
        SlewRateLimiter[] curLimiters = RobotContainer.getLimiters();
        brakeMode = DriveSubsystem.getBrakeMode();
        fieldRelative = DriveSubsystem.getFieldRelative();
        // Stored in a new object for Oblog functionality
        inputX = m_translationXSupplier.getAsDouble();
        adjustX(curLimiters);
        inputY = m_translationYSupplier.getAsDouble();
        adjustY(curLimiters);
        inputRot = m_rotationSupplier.getAsDouble();
        // adjustRot();

        if (!brakeMode) {
            DriveSubsystem.setDefending(false);
            RobotDriveTrainState = "Brake Mode Off";
            m_drivetrainSubsystem.drive(fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(inputX, inputY, inputRot,
                            m_drivetrainSubsystem.getGyroscopeRotation())
                    : new ChassisSpeeds(inputX, inputY, inputRot));
        } else {
            if (inputX != 0.0 || inputY != 0.0 || inputRot != 0.0) {
                DriveSubsystem.setDefending(false);
                RobotDriveTrainState = "Brake Mode On - Driving";
                m_drivetrainSubsystem.drive(fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(inputX, inputY, inputRot,
                                m_drivetrainSubsystem.getGyroscopeRotation())
                        : new ChassisSpeeds(inputX, inputY, inputRot));
            } else {
                RobotDriveTrainState = "Brake Mode On - Standby";
                DriveSubsystem.setDefending(true);
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

    
    public void adjustX(SlewRateLimiter[] sLimiters) {
        inputX = sLimiters[0].calculate(inputX);
        // inputX = xLimiter.calculate(inputX) *
        // DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        // inputX = xLimiter.calculate(inputX) * 4.96824;

    }

    public void adjustY(SlewRateLimiter[] sLimiters) {
        inputY = sLimiters[1].calculate(inputY);
        // inputY = yLimiter.calculate(inputY) *
        // DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        // inputY = yLimiter.calculate(inputY) * 4.96824;
    }

    public void adjustRot(SlewRateLimiter[] sLimiters) {
        inputRot = sLimiters[2].calculate(inputRot);
        // inputRot = rotLimiter.calculate(inputRot) *
        // DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        // inputRot = rotLimiter.calculate(inputRot) * (3 * Math.PI);

    }
    
    public static SlewRateLimiter[] getLimiterArray(DriveType driveType) {
        SlewRateLimiter[] ret;
        switch (driveType) {
            default:
            case NORMAL:
                ret = new SlewRateLimiter[] { new SlewRateLimiter(9), new SlewRateLimiter(9), new SlewRateLimiter(10) };
                break;
            case FAST:
                ret = new SlewRateLimiter[] { new SlewRateLimiter(2), new SlewRateLimiter(2), new SlewRateLimiter(2) };
                break;
            case NOLIMITS:
                ret = new SlewRateLimiter[] { new SlewRateLimiter(0), new SlewRateLimiter(0), new SlewRateLimiter(0) };
                break;
        }
        return ret;
    }

    public enum DriveType {
        NORMAL,
        FAST,
        NOLIMITS
    }

}
