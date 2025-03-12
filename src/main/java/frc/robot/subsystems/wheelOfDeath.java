// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.elevator.elevatorPosition;


/*
 * INITIALIZATION
 */

public class wheelOfDeath extends SubsystemBase {
  //TODO: adapt to each subsystem
  SparkMax pivotMotor;
  private SparkClosedLoopController closedLoopControllerPivot;
  private RelativeEncoder pivotEncoder;
  private AbsoluteEncoder absolutePivotEncoder;
  private SparkMaxConfig pivotConfig;

  public pivotPosition currentTargetPosition;
  /** Creates a new climber. */
  public wheelOfDeath() {
    pivotMotor = new SparkMax(PivotConstants.kCanID, MotorType.kBrushless); 
    closedLoopControllerPivot = pivotMotor.getClosedLoopController();
    pivotEncoder = pivotMotor.getEncoder();
    currentTargetPosition = pivotPosition.STOW;
    absolutePivotEncoder = pivotMotor.getAbsoluteEncoder();
    //position = climb.getPosition();

    configure();
  }

  /*
   * CONFIGURATION
   */

  private void configure(){
    pivotConfig = new SparkMaxConfig();
    pivotConfig
      .inverted(PivotConstants.kInverted)
      .smartCurrentLimit(PivotConstants.kStallLimit, PivotConstants.kFreeLimit)
      .idleMode(PivotConstants.kIdleMode); 
    pivotConfig.closedLoop
      .feedbackSensor(PivotConstants.kSensor) 
      .pidf(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD, PivotConstants.kFf) 
      .outputRange(PivotConstants.kMinOutputLimit,PivotConstants.kMaxOutputLimit);
    pivotConfig.softLimit
      .forwardSoftLimitEnabled(false)
      .forwardSoftLimit(PivotConstants.kForwardSoftLimit) 
      .reverseSoftLimitEnabled(false)
      .reverseSoftLimit(PivotConstants.kReverseSoftLimit);
    pivotConfig.encoder
      .positionConversionFactor(PivotConstants.kPositionCoversionFactor);
    pivotConfig.absoluteEncoder
    .positionConversionFactor(PivotConstants.kPositionCoversionFactor)
    .zeroOffset(PivotConstants.kOffset)
    .zeroCentered(true)
    .inverted(PivotConstants.kAbsoluteEncoderInverted);

   pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /*
   * FUCNTIONS FOR SUBSYSTEM
   */
  
  /*
   * FUCNTIONS TO SET MOTORS AND ENCODERS
   */


  public void setPivotPosition(pivotPosition stow){
    currentTargetPosition = stow;
    closedLoopControllerPivot.setReference(stow.degrees, ControlType.kPosition);
  }

  public void stop(){
   pivotMotor.stopMotor();
  }

  public void resetEncoder(){
    pivotEncoder.setPosition(0);
  }

  /*
   * FUNCTIONS TO GET VALUES
   */

  public double getPosition(){
    return absolutePivotEncoder.getPosition();
  }

  private double getPivotError() {
    return Math.abs(Math.abs(absolutePivotEncoder.getPosition()) - Math.abs(currentTargetPosition.degrees));
  }

  private boolean isAtSetpoint(){
    return (getPivotError() < PivotConstants.kTolerance);
  }

  /*
   * COMMANDS THAT DO NOT SET ANY POSITIONS
   * TODO: SEE IF WE NEED TO MOVE THIS TO ITS OWN COMMAND FILE
   */

  public Command waitUntilAtSetpoint() {
    return new WaitUntilCommand(() -> {
      // TEST FOR IF PIVOTERROR IS IN TOLERANCE OF TARGETPOSITION
      return isAtSetpoint();
    });
  }
  /*
   * COMMANDS TO SET POSITIONS ( because we can't call commands that call for the same subsystem,
   * but you can call two commands that are in the same subsystem)
   */

   public Command setPostition(pivotPosition pos){
    return runOnce(() -> {
      setPivotPosition(pos);
    });
   }
   public Command stopMotors(){
    return runOnce(()-> {
      stop();
    });
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Position", getPosition());
    SmartDashboard.putNumber("Target Pivot Position", currentTargetPosition.getDegrees());
    SmartDashboard.putBoolean("Pivot at Setpoint", isAtSetpoint());
  }

  public enum pivotPosition {
    // ENUMS FOR POSITIONS 
    STOW(0),
    L_ONE(56.25),
    L_TWO(0),
    L_THREE(0),
    L_FOUR(0),
    C_BAY(0),
    SAFE(180);

    private double degrees;
    /**Constrcutor for degrees for pivotPositions (Enum for pivot poses)
    * @param degrees
    * verticle movement in degrees
    */
    pivotPosition(double degrees) {
        this.degrees = degrees;
    }

    public double getDegrees() {
        return this.degrees;
    }
  }
}
