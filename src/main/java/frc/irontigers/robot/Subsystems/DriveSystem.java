// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.irontigers.robot.Constants;
import frc.tigerlib.subsystem.drive.DifferentialDriveSubsystem;

public class DriveSystem extends DifferentialDriveSubsystem {

  MotorController leftOne = new CANSparkMax(Constants.DriveSystemVals.LEFT_ONE, MotorType.kBrushless);
  MotorController leftTwo = new CANSparkMax(Constants.DriveSystemVals.LEFT_TWO, MotorType.kBrushless);
  MotorControllerGroup left = new MotorControllerGroup(leftOne, leftTwo);

  MotorController rightOne = new CANSparkMax(Constants.DriveSystemVals.RIGHT_ONE, MotorType.kBrushless);
  MotorController rightTwo = new CANSparkMax(Constants.DriveSystemVals.RIGHT_TWO, MotorType.kBrushless);
  MotorControllerGroup right = new MotorControllerGroup(rightOne, rightTwo);

  public int direction = 1;
  public int gear = 3 ;
  public double gearScalar;

  private AHRS gyro = new AHRS();
  

  /** Creates a new DriveSystem. */
  public DriveSystem() {
    setGyro(gyro);
    setMotors(left, right);
  }
    public void drive(double xSpeed, double rotation){
      switch(gear){
        case 0:
        gearScalar = .25;
        break;
        case 1:
        gearScalar = .5;
        break;
        case 2:
        gearScalar = .75;
        break;
        case 3:
        gearScalar = 1;
        break;
      }
    super.drive(gearScalar * xSpeed, gearScalar * rotation);
    }

  public void shiftUp(){
    if(gear < 3){
      gear++;
    }
  }

  public void shiftDown(){
    if(gear > 0){
      gear--;
    }
  }

  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected double getLeftDistance() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  protected double getRightDistance() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  protected void resetEncoders() {
    // TODO Auto-generated method stub
    
  }
}
