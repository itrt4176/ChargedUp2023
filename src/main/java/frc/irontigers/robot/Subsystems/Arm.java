// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Subsystems;

import org.yaml.snakeyaml.scanner.Constant;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.irontigers.robot.Constants;
import frc.irontigers.robot.Commands.ArmManualLengthAdjustment;
import frc.irontigers.robot.Commands.ManualArmRotation;
import frc.irontigers.robot.Constants.ArmVals;

public class Arm extends SubsystemBase {
 

  /** Creates a new Arm. */
  private WPI_TalonFX armRotatorMain;
  private WPI_TalonFX armExtender;
  private WPI_TalonFX armRotatorSub;

  private Solenoid armLock;

  private double armRotationPosition;
  private Pose2d armExtensionPosition;

  private DoubleLogEntry armPositionLog;
  private DoubleLogEntry armExtensionLog;

  private boolean isArmLocked;

  public Arm() {
    armRotatorMain = new WPI_TalonFX(Constants.ArmVals.ARM_ROTATOR_MASTER);
    armExtender = new WPI_TalonFX(Constants.ArmVals.ARM_EXTENDER);

    armRotatorSub = new WPI_TalonFX(Constants.ArmVals.ARM_ROTATOR_SUB);

    armLock = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    armRotatorSub.follow(armRotatorMain);
    armRotatorSub.setInverted(true);
    
    
    armExtender.setInverted(true);

    DataLog log = DataLogManager.getLog();{
      armPositionLog = new DoubleLogEntry(log, "arm/position");
      armExtensionLog = new DoubleLogEntry(log, "arm/extension");
    }

    armRotatorMain.setNeutralMode(NeutralMode.Brake);
    armExtender.setNeutralMode(NeutralMode.Brake);
  }

  // public WPI_TalonFX getArmExtender() {
  //   return armExtender;
  // }


  public double getArmDegrees(){
    return armRotatorMain.getSelectedSensorPosition() * Constants.ArmVals.PULSES_TO_DEGREES;
  }

  
     
  public void setRotationSpeed(double speed) {
     armRotatorMain.set(speed * 80.0 / 200.0);
  }

  public boolean isRotatorHome() {
    return armRotatorMain.getSensorCollection().isRevLimitSwitchClosed() == 1; 
  }

  public void setExtensionSpeed(double speed) {
    // if (getArmExtensionPosition() <= 20.9375){
    //   armExtender.set(speed);
    // }else{
    //   armExtender.set(0);
    // }
     armExtender.set(speed);
  }

  public double getRotatorPosition() {
    double position = armRotatorMain.getSelectedSensorPosition();
    armPositionLog.append(position);

 
    return position;
  }

  public void armLockOn() {
    armRotatorMain.setNeutralMode(NeutralMode.Coast);
    armRotatorSub.setNeutralMode(NeutralMode.Coast);
    armLock.set(true);
    isArmLocked = true;
  }

  public void armLockOff() {
    armRotatorMain.setNeutralMode(NeutralMode.Brake);
    armRotatorSub.setNeutralMode(NeutralMode.Brake);
    armLock.set(false);
    isArmLocked = false;
  }

  public double getArmExtensionPosition() {
    double extensionPosition = armExtender.getSelectedSensorPosition() * Constants.ArmVals.EXTENDER_CONVERSION_FACTOR;
    armExtensionLog.append(extensionPosition);

    return extensionPosition;
  }

  public boolean isExtensionHome() {
    return armExtender.getSensorCollection().isRevLimitSwitchClosed() == 1;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    
    SmartDashboard.putNumber("Arm Position", getArmDegrees());
    SmartDashboard.putNumber("Arm Length", getArmExtensionPosition());
    SmartDashboard.putNumber("Raw Rotator Pulses", armRotatorMain.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Arm Locked?", isArmLocked);
    SmartDashboard.putBoolean("Arm 0", getArmDegrees() >= -2.5 && getArmDegrees() <= 2.5);
    SmartDashboard.putBoolean("Arm High", getArmDegrees() >= 177.5 && getArmDegrees() <= 182.5);
    SmartDashboard.putBoolean("Arm Mid", getArmDegrees() >= 187.5 && getArmDegrees() <= 192.5);

  }

  

}
 