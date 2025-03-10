// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
public final class Constants {

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds

        public static final double kMaxSpeedMetersPerSecond = 0.6;
        public static final double kMaxAngularSpeed = 1 * Math.PI; // radians per second

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 5;
        public static final int kRearLeftDrivingCanId = 7;
        public static final int kFrontRightDrivingCanId = 3;
        public static final int kRearRightDrivingCanId = 1;

        public static final int kFrontLeftTurningCanId = 6;
        public static final int kRearLeftTurningCanId = 8;
        public static final int kFrontRightTurningCanId = 4;
        public static final int kRearRightTurningCanId = 2;

        public static final int kFrontLeftCanCoderID = 9;
        public static final int kRearLeftCanCoderID = 10;
        public static final int kFrontRightCanCoderID = 11;
        public static final int kRearRightCanCoderID = 12;

        public static final boolean kGyroReversed = false;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).

        public static final int kDrivingMotorPinionTeeth = 14;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.1016;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
    }

    public static final class OIConstants {

        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {

        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class ElevatorConstants {

        public static final int ElEVATOR_MOTOR_ID = 12;
        public static final double Orginal = 0;
        //public static final double gearRatio =  1 / 20;
        public static final double encoderSetpoint = 110; // lvel three 110 //112 is the top if 0 is the bottom
        public static final double halfEncoderSetpoint = 37; // level two
    }

    public static final class RollerConstants {

        public static final int ROLLER_MOTOR_ID = 7;
        public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
        public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
        public static final double ROLLER_EJECT_VALUE = -0.85; //-0.75 //shooting it out -
        public static final double Autonomuse_ROLLER_EJECT_VALUE = -0.21;
        public static final double ROLLER_SHOOT_VALUE = 0.2; // intakeing it in +
    }

    public static final class AlgaeConstants {

        public static final int Algae_Intake_ID = 10;
        //public static final double gearRatio = 0.0;
        public static final double encoderSetpoint = 0.215; //intake
        public static final double original = 0.13; //taken
//FIND NEW ENCODERSETPOINT FOR ALGAEPIVOT
    }

    public static final class AlageRollerConstants {

        public static final int ALGAE_ROLLER_MOTOR_ID = 11;
        public static final int ALAGE_ROLLER_MOTOR_CURRENT_LIMIT = 60;
        public static final double ALAGE_ROLLER_MOTOR_VOLTAGE_COMP = 10;
        public static final double ALAGE_ROLLER_INTAKE = -1.0;
        public static final double AlAGE_ROLLER_SHOOT = 1.0;
    }

    public static final class PivotConstants {

        public static final int PIVOT_MOTOR_ID = 9;
        // public static final double gearRatio = 1 / 75;
        public static final double intakePosition = -17;
        public static final double lvTwoAndThreeEncoderSetpoint = -8.5; //lv 2 and 3 position
        //public static final double intakePosition = -17.0;
        // public static final double lvTwoAndThreeEncoderSetpoint = -8.0; //lv 2 and 3 position og
        public static final double bottomEncoderSetpoint = -16.0; //intake position
        public static final double topEncoderSetpoint = -32.0;
    }

    public static final class OperatorConstants {

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }
}
