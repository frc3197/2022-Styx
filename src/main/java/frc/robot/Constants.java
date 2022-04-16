// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.other.extra_libraries.PIDConst;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants implements Loggable {

    public static final class dimensions {

        public static final double TRACKWIDTH = Units.inchesToMeters(23.75);
        public static final double WHEELBASE = Units.inchesToMeters(19);
        // NOTE : REAL VALUES ARE 24.25 and 19.25
    }

    public static final class subsystems {

        public static final class swerve {
            // ORDER: FL FR BL BR
            @Log
            public static final double MAX_VEL_METERS = 6380.0 / 60.0
                    * SdsModuleConfigurations.MK4_L2.getDriveReduction()
                    * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
            @Log
            public static final double MAX_ANG_VEL_RAD = MAX_VEL_METERS
                    / Math.hypot(Constants.dimensions.TRACKWIDTH / 2.0, Constants.dimensions.WHEELBASE / 2.0);

            @Log
            public static final double MAX_VOLTAGE = 12.0;
            @Log
            public static final double MAX_ANG_ACCEL = 10 * Math.PI;
            public static final boolean feildRelativeOn = true;
            public static final boolean brakeModeOn = false;
            public static final PIDConst xALIGN_PID = new PIDConst(.3, 0, .01);
            // OLD Y VALUES, .012
            public static final PIDConst yALIGN_PID = new PIDConst(.014, 0, 0);

            public static final class modInfo {

                public static final class blMod {
                    public static final int MODULE_DRIVE_MOTOR = 4;
                    public static final int MODULE_STEER_MOTOR = 5;
                    public static final int MODULE_STEER_ENCODER = 2;
                    public static final double MODULE_STEER_OFFSET = -Math.toRadians(311.221);
                }

                public static final class brMod {
                    public static final int MODULE_DRIVE_MOTOR = 6;
                    public static final int MODULE_STEER_MOTOR = 7;
                    public static final int MODULE_STEER_ENCODER = 3;
                    public static final double MODULE_STEER_OFFSET = -Math.toRadians(330.117);
                }

                public static final class flMod {
                    public static final int MODULE_DRIVE_MOTOR = 3;
                    public static final int MODULE_STEER_MOTOR = 2;
                    public static final int MODULE_STEER_ENCODER = 1;
                    public static final double MODULE_STEER_OFFSET = -Math.toRadians(122.871);
                }

                // MOD 1
                public static final class frMod {
                    public static final int MODULE_DRIVE_MOTOR = 0;
                    public static final int MODULE_STEER_MOTOR = 1;
                    public static final int MODULE_STEER_ENCODER = 0;
                    public static final double MODULE_STEER_OFFSET = -Math.toRadians(296.543);
                }
            }

        }

        public static final class H_Auto {

            public static final double TRACKWIDTH = Units.inchesToMeters(23.75);
            public static final double WHEELBASE = Units.inchesToMeters(19);
            public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.96824;
            public static final double MIN_VELOCITY_METERS_PER_SECOND = 0.125;

            public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;

            public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                    Math.hypot(TRACKWIDTH / 2.0, WHEELBASE / 2.0);
            public static final double MAX_ANGULAR_VELOCITY = Math.PI * 3; // radians per second
            public static final double MIN_ANGULAR_VELOCITY = Math.PI * 0.15; // radians per second
            public static final double MAX_ANGULAR_ACCELERATION = Math.PI * 10.0 * 2; // radians per second squared
        }

        public static final class shooter {
            public static final int shooterMotorID = 12;
            public static final int shooterEncoderA = 1;
            public static final int shooterEncoderB = 0;

            public static final double kP = 0.17215;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double shooterMaxVoltage = 12;

            public static final double kS = .44183;
            public static final double kV = .93057;
            public static final double kA = 0.43233;
            public static final double waitTime = 0;
            public static final double defaultTurnSpeed = .25;
        }

        public static final class intake {

            public static final int intakeMotorID = 9;
            public static final double intakeSpeed = .8;
            public static final String camName = "intakeCam";
            public static int armMotorID = 14;
            public static double armSpeed = .4;
        }

        public static final class climber {
            public static final int spoolMotorLeftID = 19;
            public static final int spoolMotorRightID = 17;
            public static final int armMotorLeftID = 10;
            public static final int armMotorRightID = 11;
            public static final double spoolSpeed = 1;
            public static final int armEncoderLeftA = 2;
            public static final int armEncoderLeftB = 3;
            public static final double armSpeed = .8;

            public static final double armMaxVoltage = 6;
            public static final double armRotationTolerance = 2;
            public static final PIDConst climberArmCost = new PIDConst(.015, .01, 0);
        }

        public static final class lifter {

            public static final int lifterBBChannel = 6;
            public static final int feederBBChannel = 8;

            public static final int lifterMotorID = 8;

            public static final int feederMotorID = 18;

            public static final double lifterSpeed = -.2;
            public static final double lifterShootSpeed = -.4;
            public static final double feederSpeed = 1;

        }

        public static final class hood {
            public static final int hoodMotorID = 15;
            public static final int hoodEncoderID = 4;
            // inches
            public static final int LLHeight = 36;
            public static final double HubHeight = 104;
            public static final double LLAng = 20;
            public static final double hoodSpeed = .1;
        }

    }

    public static final class outputs {
        public static final double strafe = .8;
        public static final double turnRate = .8;
    }

    public static final class auto {

        public static final class follower {
            @Log
            private static final double MAX_ANG_VEL_RAD_AUTO = 8 * Math.PI;
            //public static final TrapezoidProfile.Constraints ROT_PROFILE = new TrapezoidProfile.Constraints(subsystems.H_Auto.MAX_ANGULAR_VELOCITY, subsystems.H_Auto.MAX_ANGULAR_ACCELERATION);
            public static final TrapezoidProfile.Constraints ROT_PROFILE = new TrapezoidProfile.Constraints(subsystems.swerve.MAX_ANG_VEL_RAD, subsystems.swerve.MAX_ANG_ACCEL);

            @Log
            public static final PIDController X_PID_CONTROLLER = new PIDController(.25, 0, 0);
            @Log
            public static final PIDController Y_PID_CONTROLLER = new PIDController(.25, 0, 0);
            @Log
            public static final ProfiledPIDController ROT_PID_CONTROLLER = new ProfiledPIDController(2.8, 0, 0,
                    ROT_PROFILE);
            // DRIVING DEFAULT IS 5
            public static final double LINEAR_VELOCITY_DEFAULT = 0;
// BEST SO FAR .1 .1 3.4
        }

        public static final class startingPos {
            public static final Pose2d DEFAULT_POS = new Pose2d();

        }
    }
}