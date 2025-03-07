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
import frc.robot.Constants.ClimbConstants;

/*
 * INITIALIZATION
 */

public class coral extends SubsystemBase {
  //TODO: adapt to each subsystem
  SparkMax coralMotor;
  private SparkClosedLoopController closedLoopControllerC;
  private RelativeEncoder coralEncoder;
  private SparkMaxConfig climbConfig;

  public coralSpeed currentTargetSpeed;
  /** Creates a new climber. */
  public coral() {
    coralMotor = new SparkMax(ClimbConstants.kCanID, MotorType.kBrushless); 
    closedLoopControllerC = coralMotor.getClosedLoopController();
    coralEncoder = coralMotor.getEncoder();
    currentTargetSpeed = coralSpeed.STOW;

    configure();
  }

  /*
   * CONFIGURATION
   */

  private void configure(){
    climbConfig = new SparkMaxConfig();
    climbConfig
      .inverted(ClimbConstants.kInverted)
      .smartCurrentLimit(ClimbConstants.kStallLimit, ClimbConstants.kFreeLimit)
      .idleMode(ClimbConstants.kIdleMode); 
    climbConfig.closedLoop
      .feedbackSensor(ClimbConstants.kSensor) 
      .pidf(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD, ClimbConstants.kFf) 
      .outputRange(ClimbConstants.kMinOutputLimit,ClimbConstants.kMaxOutputLimit);
    climbConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(ClimbConstants.kForwardSoftLimit) 
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(ClimbConstants.kReverseSoftLimit);
    climbConfig.encoder
      .positionConversionFactor(ClimbConstants.kPositionCoversionFactor);

   coralMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /*
   * FUCNTIONS FOR SUBSYSTEM
   */
  
  /*
   * FUCNTIONS TO SET MOTORS AND ENCODERS
   */


  public void intake(){
    currentTargetSpeed = coralSpeed.REACH;
    closedLoopControllerC.setReference(currentTargetSpeed.speed, ControlType.kVelocity);
  }

  public void outtake(){
    currentTargetSpeed = coralSpeed.STOW;
    closedLoopControllerC.setReference(currentTargetSpeed.speed, ControlType.kPosition);
  }

  public void setSpeed(coralSpeed speed){
    currentTargetSpeed = speed;
    closedLoopControllerC.setReference(speed.speed, ControlType.kPosition);
  }

  public void stop(){
   coralMotor.stopMotor();
  }

  public void resetEncoder(){
    coralEncoder.setPosition(0);
  }

  /*
   * FUNCTIONS TO GET VALUES
   */

  public double getSpeed(){
    return coralEncoder.getVelocity();
  }

  private double getSpeedError() {
    return Math.abs(Math.abs(coralEncoder.getVelocity()) - Math.abs(currentTargetSpeed.speed));
  }

  private boolean isAtSpeed(){
    return (getSpeedError() < ClimbConstants.kTolerance);
  }

  /*
   * COMMANDS THAT DO NOT SET ANY POSITIONS
   * TODO: SEE IF WE NEED TO MOVE THIS TO ITS OWN COMMAND FILE
   */

  public Command waitUntilAtSpeed() {
    return new WaitUntilCommand(() -> {
      // TEST FOR IF ELEVATORERROR IS IN TOLERANCE OF TARGETPOSITION
      return isAtSpeed();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake velocity", getSpeed());
    SmartDashboard.putNumber("Target intake velocity", currentTargetSpeed.getSpeed());
    SmartDashboard.putBoolean("Climb at Setpoint", isAtSpeed());
  }

  public enum coralSpeed {
    // ENUMS FOR POSITIONS 
    STOW(4.16),
    REACH(56.25);

    private double speed;
    /**Constrcutor for speed for coralSpeeds (Enum for climb poses)
    * @param speed
    * verticle movement in speed
    */
    coralSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return this.speed;
    }
  }
}