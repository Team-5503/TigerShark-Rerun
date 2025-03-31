package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.generated.TunerConstants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;


/** Add your docs here. */
public class Constants {
    public static class AutoConstants {

    }

    public static class ElevatorConstants { //TODO: change and tune values
        /*
         * ids
         */
        public static final int kL1CanID = 13;
        public static final int kL2CanID = 14;
        /*
         * config
         */
        public static final boolean kL1Inverted = true;
        public static final boolean kL2Inverted = true;
        public static final int kStallLimit = 70;
        public static final int kFreeLimit = 45;
        public static final IdleMode kIdleMode = IdleMode.kBrake; // TODO: change to brake
        /*
         * closed loop
         */
        public static final FeedbackSensor kSensor = FeedbackSensor.kPrimaryEncoder;
        public static final double kPL1 = .017;
        public static final double kIL1 = 0;
        public static final double kDL1 = .007;
        public static final double kFfL1 = 0;
        public static final double kPL2 = .017;
        public static final double kIL2 = 0;
        public static final double kDL2 = .007;
        public static final double kFfL2 = 0;
        public static final double kMinOutputLimit = -.8;
        public static final double kMaxOutputLimit = .8;
        /*
         * soft limit
         */
        public static final double kL1ForwardSoftLimit = 50;
        public static final double kL1ReverseSoftLimit = -10;
        public static final double kL2ForwardSoftLimit = 35;
        public static final double kL2ReverseSoftLimit = -10;
        /*
         * encoder
         */
        public static final double kPositionCoversionFactor = 1;
        public static final double kTolerance = 1;
    }

    public static class PivotConstants {//TODO: change and tune values
        /*
         * ids
         */
        public static final int kCanID = 15;
        /*
         * config
         */
        public static final boolean kInverted = true;
        public static final int kStallLimit = 70;
        public static final int kFreeLimit = 50;
        public static final IdleMode kIdleMode = IdleMode.kBrake; // TODO: change to brake

        public static final double kOffset = 0;
        public static final boolean kAbsoluteEncoderInverted = false;
        /*
         * closed loop
         */
        public static final FeedbackSensor kSensor = FeedbackSensor.kAbsoluteEncoder;
        public static final double kP = .008;
        public static final double kI = 0;
        public static final double kD = .004;
        public static final double kFf = 0;
        public static final double kMinOutputLimit = -.16;
        public static final double kMaxOutputLimit = .18;
        /*
         * soft limit
         */
        public static final double kForwardSoftLimit = 130;
        public static final double kReverseSoftLimit = -60;
        /*
         * encoder
         */
        public static final double kPositionCoversionFactor = 360;
        public static final double kTolerance = 5;
    }

    public static class CoralConstants { //TODO: change and tune values
        /*
         * ids
         */
        public static final int kCanID = 16;
        public static final int kSensorID = 19;
        /*
         * config
         */
        public static final boolean kInverted = true;
        public static final int kStallLimit = 80;
        public static final int kFreeLimit = 40;
        public static final IdleMode kIdleMode = IdleMode.kCoast; // TODO: change to brake
        /*
         * closed loop
         */
        public static final FeedbackSensor kSensor = FeedbackSensor.kPrimaryEncoder;
        public static final double kP = .02;
        public static final double kI = 0;
        public static final double kD = .05;
        public static final double kFf = 0;
        public static final double kMinOutputLimit = -.8;
        public static final double kMaxOutputLimit = .8;
        /*
         * soft limit
         */
        public static final double kForwardSoftLimit = 63;
        public static final double kReverseSoftLimit = -1;
        /*
         * encoder
         */
        public static final double kPositionCoversionFactor = 1;
        public static final double kTolerance = 40; // TODO: change if needed
    }

    public static class AlgaeConstants { //TODO: change and tune values
        /*
         * ids
         */
        public static final int kCanID = 17;
        /*
         * config
         */
        public static final boolean kInverted = true;
        public static final int kStallLimit = 80;
        public static final int kFreeLimit = 40;
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        /*
         * closed loop
         */
        public static final FeedbackSensor kSensor = FeedbackSensor.kPrimaryEncoder;
        public static final double kP = .25;
        public static final double kI = 0;
        public static final double kD = .05;
        public static final double kFf = 0;
        public static final double kMinOutputLimit = -.8;
        public static final double kMaxOutputLimit = .8;
        /*
         * soft limit
         */
        public static final double kForwardSoftLimit = 63;
        public static final double kReverseSoftLimit = -1;
        /*
         * encoder
         */
        public static final double kPositionCoversionFactor = 1;
        public static final double kTolerance = 5;
    }

    public static class ClimbConstants {
        /*
         * ids
         */
        public static final int kCanID = 41;
        /*
         * config
         */
        public static final boolean kInverted = true;
        public static final int kStallLimit = 80;
        public static final int kFreeLimit = 40;
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        /*
         * closed loop
         */
        public static final FeedbackSensor kSensor = FeedbackSensor.kPrimaryEncoder;
        public static final double kP = .15;
        public static final double kI = 0;
        public static final double kD = .05;
        public static final double kFf = 0;
        public static final double kMinOutputLimit = -.8;
        public static final double kMaxOutputLimit = .8;
        /*
         * soft limit
         */
        public static final double kForwardSoftLimit = 63;
        public static final double kReverseSoftLimit = -1;
        /*
         * encoder
         */
        public static final double kPositionCoversionFactor = 1;
        public static final double kTolerance = 5;
    }
}
