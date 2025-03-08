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

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;

/*
 * INITIALIZATION
 */

public class coral extends SubsystemBase {
  //TODO: adapt to each subsystem
  SparkMax coralMotor;
  LaserCan coralDetect;

  private SparkClosedLoopController closedLoopControllerC;
  private RelativeEncoder coralEncoder;
  private SparkMaxConfig coralConfig;

  public coralSpeed currentTargetSpeed;
  /** Creates a new coral intake. */
  public coral() {
    coralMotor = new SparkMax(CoralConstants.kCanID, MotorType.kBrushless); 
    coralDetect = new LaserCan(CoralConstants.kSensorID);
    closedLoopControllerC = coralMotor.getClosedLoopController();
    coralEncoder = coralMotor.getEncoder();
    currentTargetSpeed = coralSpeed.STOP;

    configure();
  }

  /*
   * CONFIGURATION
   */

  private void configure(){
    coralConfig = new SparkMaxConfig();
    coralConfig
      .inverted(CoralConstants.kInverted)
      .smartCurrentLimit(CoralConstants.kStallLimit, CoralConstants.kFreeLimit)
      .idleMode(CoralConstants.kIdleMode); 
    coralConfig.closedLoop
      .feedbackSensor(CoralConstants.kSensor) 
      .pidf(CoralConstants.kP, CoralConstants.kI, CoralConstants.kD, CoralConstants.kFf) 
      .outputRange(CoralConstants.kMinOutputLimit,CoralConstants.kMaxOutputLimit);
    coralConfig.softLimit
      .forwardSoftLimitEnabled(false)
      .forwardSoftLimit(CoralConstants.kForwardSoftLimit) 
      .reverseSoftLimitEnabled(false)
      .reverseSoftLimit(CoralConstants.kReverseSoftLimit);
    coralConfig.encoder
      .positionConversionFactor(CoralConstants.kPositionCoversionFactor);

   coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
// TODO: change lasercan config values to constants
   try {
    coralDetect.setRangingMode(LaserCan.RangingMode.SHORT);
    coralDetect.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
    coralDetect.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e){
      System.out.println("configuration failed: " + e);
    }
  }

  /*
   * FUCNTIONS FOR SUBSYSTEM
   */
  
  /*
   * FUCNTIONS TO SET MOTORS AND ENCODERS
   */


  public void intake(){
    currentTargetSpeed = coralSpeed.INTAKE;
    closedLoopControllerC.setReference(currentTargetSpeed.speed, ControlType.kDutyCycle);
  }

  public void outtake(){
    currentTargetSpeed = coralSpeed.OUTTAKE;
    closedLoopControllerC.setReference(currentTargetSpeed.speed, ControlType.kDutyCycle);
  }

  public void setSpeed(coralSpeed speed){
    currentTargetSpeed = speed;
    closedLoopControllerC.setReference(speed.speed, ControlType.kDutyCycle);
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
    return (getSpeedError() < CoralConstants.kTolerance);
  }

  private LaserCan.Measurement getMeasurement() {
    return coralDetect.getMeasurement();
  }

  private boolean hasCoral(){
    return (getMeasurement().distance_mm < CoralConstants.kTolerance);
  }

  /*
   * COMMANDS THAT DO NOT SET ANYTHING
   * TODO: SEE IF WE NEED TO MOVE THIS TO ITS OWN COMMAND FILE
   */

  public Command waitUntilAtSpeed() {
    return new WaitUntilCommand(() -> {
      // TEST FOR IF VELOCITYERROR IS IN TOLERANCE OF TARGETVELOCITY
      return isAtSpeed();
    });
  }

  public Command waitUntilHasCoral() {
    return new WaitUntilCommand(() -> {
      // TEST FOR IF VELOCITYERROR IS IN TOLERANCE OF TARGETVELOCITY
      return hasCoral();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral intake voltage", getSpeed());
    SmartDashboard.putNumber("Target intake voltage", currentTargetSpeed.getSpeed());
    SmartDashboard.putBoolean("Coral Intake at target voltage", isAtSpeed());
  }

  public enum coralSpeed {
    // ENUMS FOR VOLTAGES 
    INTAKE(.15),
    STOP(0),
    OUTTAKE(.6);

    private double speed;
    /**Constrcutor for speed for coralSpeeds (Enum for voltage)
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