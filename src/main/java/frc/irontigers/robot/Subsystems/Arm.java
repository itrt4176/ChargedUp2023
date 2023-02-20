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

  private Pose2d armRotationPosition;

  private DoubleLogEntry armPositionLog;
  private DoubleLogEntry armExtensionLog;

  public Arm() {
    armRotator = new WPI_TalonFX(Constants.ArmVals.ARM_ROTATOR);
    armExtender = new WPI_TalonFX(Constants.ArmVals.ARM_EXTENDER);

    armRotationPosition = new Pose2d();
    

    DataLog log = DataLogManager.getLog();{
      armPositionLog = new DoubleLogEntry(log, "arm/position");
      armExtensionLog = new DoubleLogEntry(log, "arm/extension");
    }

    

    

    armRotator.setNeutralMode(NeutralMode.Brake);
    armExtender.setNeutralMode(NeutralMode.Brake);
  }

  public Pose2d getArmDegrees(){
    return armRotationPosition;
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
    getRotatorPosition();
    getArmDegrees();
  
    Pose2d armPos = getArmDegrees();
    

    
    SmartDashboard.putNumber("Arm Position", getRotatorPosition());
  }

  
}
