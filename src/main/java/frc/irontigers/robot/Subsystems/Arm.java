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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.irontigers.robot.Constants;
import frc.irontigers.robot.Constants.ArmVals;

public class Arm extends SubsystemBase {
 

  /** Creates a new Arm. */
  private WPI_TalonFX armRotator;
  private WPI_TalonFX armExtender;

  private DoubleLogEntry armPositionLog;
  private DoubleLogEntry armExtensionLog;

  public Arm() {
    armRotator = new WPI_TalonFX(Constants.ArmVals.ARM_ROTATOR);
    armExtender = new WPI_TalonFX(Constants.ArmVals.ARM_EXTENDER);
    
    DataLog log = DataLogManager.getLog();{
      armPositionLog = new DoubleLogEntry(log, "arm/position");
      armExtensionLog = new DoubleLogEntry(log, "arm/extension");
    }

    armRotator.setNeutralMode(NeutralMode.Brake);
    armExtender.setNeutralMode(NeutralMode.Brake);
  }

  public double getArmDegrees(){
    double armPose2d = armRotator.getSelectedSensorPosition() * Constants.ArmVals.PULSES_TO_DEGREES;
    return armPose2d;
  }
     
  public void setRotationSpeed(double speed) {
     armRotator.set(speed);
  }

  public void setExtensionSpeed(double speed) {
    armExtender.set(speed);
  }

  public double getRotatorPosition() {
    double position = armRotator.getSelectedSensorPosition();
    armPositionLog.append(position);
 
    return position;
  }


   public double getArmExtensionPosition() {
     double extensionPosition = armExtender.getSelectedSensorPosition();
    armExtensionLog.append(extensionPosition);
    
    return extensionPosition;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  
    double armPos = getRotatorPosition() * Constants.ArmVals.PULSES_TO_DEGREES;
    armPositionLog.append(armPos);

    SmartDashboard.putNumber("Arm Position", armPos);

    
     
  }

  
}
