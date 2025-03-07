package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.generated.TunerConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class Constants {
    public static class AutoConstants {

    }

    public static class ElevatorConstants { //TODO: change and tune values
        /*
         * ids
         */
        public static final int kL1CanID = 41;
        public static final int kL2CanID = 41;
        /*
         * config
         */
        public static final boolean kL1Inverted = true;
        public static final boolean kL2Inverted = true;
        public static final int kStallLimit = 60;
        public static final int kFreeLimit = 30;
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
        public static final double kL1ForwardSoftLimit = 63;
        public static final double kL1ReverseSoftLimit = -1;
        public static final double kL2ForwardSoftLimit = 63;
        public static final double kL2ReverseSoftLimit = -1;
        /*
         * encoder
         */
        public static final double kPositionCoversionFactor = 1;
        public static final double kTolerance = 2;
    }

    public static class PivotConstants {//TODO: change and tune values
        /*
         * ids
         */
        public static final int kCanID = 41;
        /*
         * config
         */
        public static final boolean kInverted = true;
        public static final int kStallLimit = 50;
        public static final int kFreeLimit = 30;
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        /*
         * closed loop
         */
        public static final FeedbackSensor kSensor = FeedbackSensor.kAbsoluteEncoder;
        public static final double kP = .05;
        public static final double kI = 0;
        public static final double kD = .02;
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

    public static class CoralConstants { //TODO: change and tune values
        /*
         * ids
         */
        public static final int kCanID = 41;
        public static final int kSensorID = 19;
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
        public static final double kP = .2;
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
        public static final double kTolerance = 15; // TODO: change if needed
    }

    public static class AlgaeConstants { //TODO: change and tune values
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
