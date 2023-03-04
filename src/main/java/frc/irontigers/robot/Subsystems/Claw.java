// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.irontigers.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;


public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private WPI_TalonFX claw;
  private Solenoid clawPnuematic;


  private DoubleLogEntry clawPositionLog;
  

  public Claw() {
    // claw = new WPI_TalonFX(Constants.ClawVals.CLAW);
    
    clawPnuematic = new Solenoid(PneumaticsModuleType.REVPH , 1);

    DataLog log = DataLogManager.getLog();{
      clawPositionLog = new DoubleLogEntry(log, "claw/position");
      
    }

    // claw.setNeutralMode(NeutralMode .Brake);
  }

  // public void setClawOneSpeed(double speed) {
  //   claw.set(speed);
  // }

  public void setClawStateTrue(){
    clawPnuematic.set(true);
  }

  public void setClawStateFalse(){
    clawPnuematic.set(false);
  }
  

 

  // public double[] getClawPositions() {
  //   double[] clawPositions = {claw.getSelectedSensorPosition()};
  //   clawPositionLog.append(clawPositions[0]);
  //   clawPositionLog.append(clawPositions[1]);
  //   return clawPositions;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
