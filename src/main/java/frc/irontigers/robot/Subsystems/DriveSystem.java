// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.irontigers.robot.Constants.DriveSystemVals.*;
import frc.tigerlib.subsystem.drive.DifferentialDriveSubsystem;

public class DriveSystem extends DifferentialDriveSubsystem {

  private CANSparkMax leftOne = new CANSparkMax(LEFT_ONE, MotorType.kBrushless);
  private CANSparkMax leftTwo = new CANSparkMax(LEFT_TWO, MotorType.kBrushless);
  private MotorControllerGroup left = new MotorControllerGroup(leftOne, leftTwo);

  private RelativeEncoder leftOneEncoder = leftOne.getEncoder();
  private RelativeEncoder leftTwoEncoder = leftTwo.getEncoder();

  private CANSparkMax rightOne = new CANSparkMax(RIGHT_ONE, MotorType.kBrushless);
  private CANSparkMax rightTwo = new CANSparkMax(RIGHT_TWO, MotorType.kBrushless);
  private MotorControllerGroup right = new MotorControllerGroup(rightOne, rightTwo);

  private RelativeEncoder rightOneEncoder = rightOne.getEncoder();
  private RelativeEncoder rightTwoEncoder = rightTwo.getEncoder();

  public int direction = 1;
  public int gear = 2;
  public double gearScalar;


  private DataLog log;
  private IntegerLogEntry gearLog;
  private DoubleLogEntry gearScalarLog;

  private DoubleLogEntry odoxLog;
  private DoubleLogEntry odoRotationLog;

  private AHRS gyro = new AHRS();

  private DifferentialDriveKinematics kinematics;

  /** Creates a new DriveSystem. */
  public DriveSystem() {
    leftOne.restoreFactoryDefaults();
    leftTwo.restoreFactoryDefaults();
    rightOne.restoreFactoryDefaults();
    rightTwo.restoreFactoryDefaults();

    leftOne.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftTwo.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightOne.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightTwo.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftOne.burnFlash();
    rightOne.burnFlash();
    leftTwo.burnFlash();
    rightTwo.burnFlash();
    setGyro(gyro);
    setMotors(left, right);

    leftOneEncoder.setPositionConversionFactor(PULSES_TO_DISTANCE_METER);
    rightOneEncoder.setPositionConversionFactor(-PULSES_TO_DISTANCE_METER);
    
    leftOneEncoder.setVelocityConversionFactor(PULSES_TO_DISTANCE_METER / 60.0);
    rightOneEncoder.setVelocityConversionFactor(-PULSES_TO_DISTANCE_METER / 60.0);
    resetEncoders();

    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

    log = DataLogManager.getLog();
    gearLog = new IntegerLogEntry(log, "drive/gear");
    gearScalarLog = new DoubleLogEntry(log, "drive/gearScalar");

    odoxLog = new DoubleLogEntry(log, "drive/odometer/x");
    odoRotationLog = new DoubleLogEntry(log, "drive/odometer/rotation");
  }

  public void drive(double xSpeed, double rotation) {
    switch (gear) {
      case 0:
        gearScalar = .35;
        break;
      case 1:
        gearScalar = .5;
        break;
      case 2:
        gearScalar = .65;
        break;
      case 3:
        gearScalar = .8;
        break;
    }

    super.drive(gearScalar * xSpeed, gearScalar * rotation);
    gearScalarLog.append(gearScalar);
  }

  public void shiftUp(){
    if(gear < 3){
      gear++;
    }
    gearLog.append(gear);
  }

  public void shiftDown(){
    if(gear > 0){
      gear--;
    }
    gearLog.append(gear);
  }

  public void setIdleMode(CANSparkMax.IdleMode idleMode) {
    leftOne.setIdleMode(idleMode);
    leftTwo.setIdleMode(idleMode);
    rightOne.setIdleMode(idleMode);
    rightTwo.setIdleMode(idleMode);
  
  }

  /**
   * @return the kinematics
   */
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftOneEncoder.getVelocity(), rightOneEncoder.getVelocity());
  }

  public void voltageDrive(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
    drive.feed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();

    Pose2d pos = getRobotPosition();
    odoxLog.append(pos.getX());
    odoRotationLog.append(pos.getRotation().getDegrees());

    SmartDashboard.putNumber("Robot X", getRobotPosition().getX());
    SmartDashboard.putNumber("Left", leftOneEncoder.getPosition());
    SmartDashboard.putNumber("Right", rightOneEncoder.getPosition());
  }

  @Override
  protected double getLeftDistance() {
    // TODO Auto-generated method stub
    return leftOneEncoder.getPosition();
  }

  @Override
  protected double getRightDistance() {
    // TODO Auto-generated method stub
    return rightOneEncoder.getPosition();
  
  }

  @Override
  protected void resetEncoders() {
    // TODO Auto-generated method stub
    leftOneEncoder.setPosition(0);
    leftTwoEncoder.setPosition(0);
    rightOneEncoder.setPosition(0);
    rightTwoEncoder.setPosition(0);
  }
}
