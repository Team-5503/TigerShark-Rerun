// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

/*
 * INITIALIZATION
 */

public class elevator extends SubsystemBase {
  //TODO: adapt to each subsystem
  SparkMax eleMotorL1, eleMotorL2;
  private SparkClosedLoopController closedLoopControllerEleL1, closedLoopControllerEleL2;
  private RelativeEncoder eleEncoderL1, eleEncoderL2;
  private SparkMaxConfig eleConfigL1, eleConfigL2;

  public elevatorPosition currentTargetPosition;
  /** Creates a new elevator. */
  public elevator() {
    eleMotorL1 = new SparkMax(ElevatorConstants.kL1CanID, MotorType.kBrushless); 
    closedLoopControllerEleL1 = eleMotorL1.getClosedLoopController();
    eleEncoderL1 = eleMotorL1.getEncoder();
    currentTargetPosition = elevatorPosition.STOW;

    eleMotorL2 = new SparkMax(ElevatorConstants.kL2CanID, MotorType.kBrushless); 
    closedLoopControllerEleL2 = eleMotorL2.getClosedLoopController();
    eleEncoderL2 = eleMotorL2.getEncoder();
    currentTargetPosition = elevatorPosition.STOW;
    //position = climb.getPositionL1();

    configure();
  }

  /*
   * CONFIGURATION
   */

  private void configure(){
    eleConfigL1 = new SparkMaxConfig();
    eleConfigL1
      .inverted(ElevatorConstants.kL1Inverted)
      .smartCurrentLimit(ElevatorConstants.kStallLimit, ElevatorConstants.kFreeLimit)
      .idleMode(ElevatorConstants.kIdleMode); 
    eleConfigL1.closedLoop
      .feedbackSensor(ElevatorConstants.kSensor) 
      .pidf(ElevatorConstants.kPL1, ElevatorConstants.kIL1, ElevatorConstants.kDL1, ElevatorConstants.kFfL1) 
      .outputRange(ElevatorConstants.kMinOutputLimit,ElevatorConstants.kMaxOutputLimit);
    eleConfigL1.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(ElevatorConstants.kL1ForwardSoftLimit) 
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(ElevatorConstants.kL1ReverseSoftLimit);
    eleConfigL1.encoder
      .positionConversionFactor(ElevatorConstants.kPositionCoversionFactor);

   eleMotorL1.configure(eleConfigL1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

   eleConfigL2 = new SparkMaxConfig();
   eleConfigL2
     .inverted(ElevatorConstants.kL2Inverted)
     .smartCurrentLimit(ElevatorConstants.kStallLimit, ElevatorConstants.kFreeLimit)
     .idleMode(ElevatorConstants.kIdleMode); 
   eleConfigL2.closedLoop
     .feedbackSensor(ElevatorConstants.kSensor) 
     .pidf(ElevatorConstants.kPL2, ElevatorConstants.kIL2, ElevatorConstants.kDL2, ElevatorConstants.kFfL2) 
     .outputRange(ElevatorConstants.kMinOutputLimit,ElevatorConstants.kMaxOutputLimit);
   eleConfigL2.softLimit
     .forwardSoftLimitEnabled(true)
     .forwardSoftLimit(ElevatorConstants.kL2ForwardSoftLimit) 
     .reverseSoftLimitEnabled(true)
     .reverseSoftLimit(ElevatorConstants.kL2ReverseSoftLimit);
   eleConfigL2.encoder
     .positionConversionFactor(ElevatorConstants.kPositionCoversionFactor);

  eleMotorL2.configure(eleConfigL2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /*
   * FUCNTIONS FOR SUBSYSTEM
   */
  
  /*
   * FUCNTIONS TO SET MOTORS AND ENCODERS
   */


  // public void setReach(){
  //   currentTargetPosition = elevatorPosition.REACH;
  //   closedLoopControllerEleL1.setReference(currentTargetPosition.l1rotations, ControlType.kPosition);
  // }

  // public void setStow(){
  //   currentTargetPosition = elevatorPosition.STOW;
  //   closedLoopControllerEleL1.setReference(currentTargetPosition.l1rotations, ControlType.kPosition);
  // }

  public void setPosition(elevatorPosition position){
    currentTargetPosition = position;
    closedLoopControllerEleL1.setReference(position.l1rotations, ControlType.kPosition);
    closedLoopControllerEleL2.setReference(position.l2rotations, ControlType.kPosition);
  }

  public void stop(){
   eleMotorL1.stopMotor();
   eleMotorL2.stopMotor();
  }

  public void resetEncoders(){
    eleEncoderL1.setPosition(0);
    eleEncoderL2.setPosition(0);
  }

  /*
   * FUNCTIONS TO GET VALUES
   */

  public double getPositionL1(){
    return eleEncoderL1.getPosition();
  }

  public double getPositionL2(){
    return eleEncoderL2.getPosition();
  }

  private double getClimbErrorL1() {
    return Math.abs(Math.abs(eleEncoderL1.getPosition()) - Math.abs(currentTargetPosition.l1rotations));
  }

  private boolean isAtSetpointL1(){
    return (getClimbErrorL1() < ElevatorConstants.kTolerance);
  }

  private double getClimbErrorL2() {
    return Math.abs(Math.abs(eleEncoderL2.getPosition()) - Math.abs(currentTargetPosition.l2rotations));
  }

  private boolean isAtSetpointL2(){
    return (getClimbErrorL2() < ElevatorConstants.kTolerance);
  }

  /*
   * COMMANDS THAT DO NOT SET ANY POSITIONS
   * TODO: SEE IF WE NEED TO MOVE THIS TO ITS OWN COMMAND FILE
   */

  public Command waitUntilAtSetpoint() {
    return new WaitUntilCommand(() -> {
      // TEST FOR IF ELEVATORERROR IS IN TOLERANCE OF TARGETPOSITION
      return isAtSetpointL1() && isAtSetpointL2();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("L1 Position", getPositionL1());
    SmartDashboard.putNumber("L2 Position", getPositionL2());
    SmartDashboard.putNumber("Target L1 Position", currentTargetPosition.getRotationsL1());
    SmartDashboard.putNumber("Target L2 Position", currentTargetPosition.getRotationsL2());
    SmartDashboard.putBoolean("L1 at Setpoint", isAtSetpointL1());
    SmartDashboard.putBoolean("L2 at Setpoint", isAtSetpointL2());
    SmartDashboard.putBoolean("elevator at Setpoint", (isAtSetpointL1() && isAtSetpointL2()));
  }

  public enum elevatorPosition {
    // ENUMS FOR POSITIONS 
    STOW(0,0),
    L_ONE(0,0),
    L_TWO(0,0),
    L_THREE(0,0),
    L_FOUR(0,0),
    C_BAY(0,0);

    private double l1rotations, l2rotations;
    /**Constrcutor for rotations for climbPositions (Enum for climb poses)
    * @param rotations
    * verticle movement in rotations
    */
    elevatorPosition(double l1rotations, double l2rotations) {
        this.l1rotations = l1rotations;
        this.l2rotations = l2rotations;
    }

    public double getRotationsL1() {
        return this.l1rotations;
    }

    public double getRotationsL2() {
      return this.l2rotations;
  }
  }
}
