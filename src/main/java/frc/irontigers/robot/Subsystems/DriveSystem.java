// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.irontigers.robot.Constants.DriveVals.*;

import org.yaml.snakeyaml.scanner.Constant;

import frc.irontigers.robot.Constants;
import frc.irontigers.robot.Commands.CommandJoystickDrive;
import frc.tigerlib.subsystem.drive.DifferentialDriveSubsystem;

public class DriveSystem extends DifferentialDriveSubsystem {

  private CANSparkMax leftOne = new CANSparkMax(LEFT_ONE, MotorType.kBrushless);
  private CANSparkMax leftTwo = new CANSparkMax(LEFT_TWO, MotorType.kBrushless);
  private MotorControllerGroup left = new MotorControllerGroup(leftOne, leftTwo);

  private RelativeEncoder leftOneEncoder = leftOne.getEncoder();
  private SimDeviceSim leftEncoderSim;
  private SimDouble leftEncoderPosSim;
  private SimDouble leftEncoderVelSim;
  private double leftInputVoltSim;

  private CANSparkMax rightOne = new CANSparkMax(RIGHT_ONE, MotorType.kBrushless);
  private CANSparkMax rightTwo = new CANSparkMax(RIGHT_TWO, MotorType.kBrushless);
  private MotorControllerGroup right = new MotorControllerGroup(rightOne, rightTwo);

  private RelativeEncoder rightOneEncoder = rightOne.getEncoder();
  private SimDeviceSim rightEncoderSim;
  private SimDouble rightEncoderPosSim;
  private SimDouble rightEncoderVelSim;
  private double rightInputVoltSim;

  private DifferentialDrivetrainSim driveSim;
  private REVPhysicsSim motorSim;

  private int gear = 2;
  private double gearScalar;


  private DataLog log;
  private IntegerLogEntry gearLog;
  private DoubleLogEntry gearScalarLog;

  private DoubleLogEntry odoxLog;
  private DoubleLogEntry odoRotationLog;

  private AHRS gyro = new AHRS();
  private SimDeviceSim navXSim;
  private SimDouble yawSim;

  private DifferentialDriveKinematics kinematics;

  //private Field2d field;

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

    leftOne.setSmartCurrentLimit(50);
    leftTwo.setSmartCurrentLimit(50);
    rightOne.setSmartCurrentLimit(50);
    rightTwo.setSmartCurrentLimit(50);

    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    leftOne.burnFlash();
    rightOne.burnFlash();
    leftTwo.burnFlash();
    rightTwo.burnFlash();

    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    if (RobotBase.isSimulation()) {
      driveSim = new DifferentialDrivetrainSim(
        LinearSystemId.identifyDrivetrainSystem(Constants.DriveVals.V, Constants.DriveVals.A, 3.4766, 0.43912),
        DCMotor.getNEO(2),
        DifferentialDrivetrainSim.KitbotGearing.k5p95.value,
        Constants.DriveVals.TRACK_WIDTH,
        Units.inchesToMeters(2.901),
        null
      );

      motorSim = REVPhysicsSim.getInstance();
      motorSim.addSparkMax(leftOne, DCMotor.getNEO(2));
      // motorSim.addSparkMax(leftTwo, DCMotor.getNEO(1));
      motorSim.addSparkMax(rightOne, DCMotor.getNEO(2));
      // motorSim.addSparkMax(rightTwo, DCMotor.getNEO(1));

      leftEncoderSim = new SimDeviceSim("SPARK MAX ", LEFT_ONE);
      leftEncoderPosSim = leftEncoderSim.getDouble("Position");
      leftEncoderVelSim = leftEncoderSim.getDouble("Velocity");
      leftInputVoltSim = 0.0;

      rightEncoderSim = new SimDeviceSim("SPARK MAX ", RIGHT_ONE);
      rightEncoderPosSim = rightEncoderSim.getDouble("Position");
      rightEncoderVelSim = rightEncoderSim.getDouble("Velocity");
      rightInputVoltSim = 0.0;

      navXSim = new SimDeviceSim("navX-Sensor", 0);
      yawSim = navXSim.getDouble("Yaw");
    } else {
      driveSim = null;

      motorSim = null;

      leftEncoderSim = null;
      leftEncoderPosSim = null;
      leftEncoderVelSim = null;
      leftInputVoltSim = 0.0;

      rightEncoderSim = null;
      rightEncoderPosSim = null;
      rightEncoderVelSim = null;
      rightInputVoltSim = 0.0;

      navXSim = null;
      yawSim = null;
    }

    setGyro(gyro);
    setMotors(left, right);

    resetEncoders();

    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

    //field = new Field2d();
    SmartDashboard.putData("Field", gameField);

    log = DataLogManager.getLog();
    gearLog = new IntegerLogEntry(log, "drive/gear");
    gearScalarLog = new DoubleLogEntry(log, "drive/gearScalar");

    odoxLog = new DoubleLogEntry(log, "drive/odometer/x");
    odoRotationLog = new DoubleLogEntry(log, "drive/odometer/rotation");
  }

  public void simulationPeriodic() {
    motorSim.run();
    driveSim.setInputs(leftInputVoltSim, rightInputVoltSim); //figure out of right side is inverted

    driveSim.update(0.02);

    leftEncoderPosSim.set(driveSim.getLeftPositionMeters() / (ROTATIONS_TO_METERS * 1.0));
    leftEncoderVelSim.set(driveSim.getLeftVelocityMetersPerSecond() / (ROTATIONS_TO_METERS * 60.0));

    rightEncoderPosSim.set(-driveSim.getRightPositionMeters() / ROTATIONS_TO_METERS);
    rightEncoderVelSim.set(-driveSim.getRightVelocityMetersPerSecond() / (ROTATIONS_TO_METERS * 60.0));

    yawSim.set(driveSim.getHeading().getDegrees());
  };

  public void setGear(int gear) {
    this.gear = MathUtil.clamp(gear, 0, 3);
    gearLog.append(gear);
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
      case 4:
        gearScalar = 1.0;
    }
    

    super.drive(gearScalar * xSpeed, gearScalar * rotation);
    gearScalarLog.append(gearScalar);
  }
  public void twoStickDrive(double leftSpeed, double rightSpeed){
    super.drive(leftSpeed, rightSpeed);

    if (RobotBase.isSimulation()) {
      leftInputVoltSim = leftOne.get() * RobotController.getInputVoltage();
      rightInputVoltSim = -rightOne.get() * RobotController.getInputVoltage();
    }
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
    if (RobotBase.isSimulation()) {
      return new DifferentialDriveWheelSpeeds(driveSim.getLeftVelocityMetersPerSecond(), driveSim.getRightVelocityMetersPerSecond());
    }

    return new DifferentialDriveWheelSpeeds(leftOneEncoder.getVelocity() * ROTATIONS_TO_METERS / 60.0, -rightOneEncoder.getVelocity() * ROTATIONS_TO_METERS / 60.0);
  }

  

  public AHRS getGyro() {
    return gyro;
  }

  public void voltageDrive(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
    drive.feed();
    SmartDashboard.putNumber("Left Output Volts", leftVolts);
    SmartDashboard.putNumber("Right Output Volts", rightVolts);

    if (RobotBase.isSimulation()) {
      leftInputVoltSim = leftVolts;
      rightInputVoltSim = rightVolts;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();

    if (DriverStation.isAutonomous()) {
      setStandard();
    } else {
      setStandard();
    }

    // odometer.update(null, leftEncoderSim.getDistance(), rightEncoderSim.getDistance());
    gameField.setRobotPose(odometer.getPoseMeters());

    Pose2d pos = getRobotPosition();
    odoxLog.append(pos.getX());
    odoRotationLog.append(pos.getRotation().getDegrees());
    //gameField.setRobotPose(pos);

    SmartDashboard.putNumber("Robot X", getRobotPosition().getX());
    SmartDashboard.putNumber("Left", getLeftDistance());
    SmartDashboard.putNumber("Right", getRightDistance());
    

    SmartDashboard.putNumber("Roll", gyro.getRoll());

    DifferentialDriveWheelSpeeds velocity = getWheelSpeeds();
    SmartDashboard.putNumber("Left Velocity (mps)", velocity.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Velocity (mps)", velocity.rightMetersPerSecond);
  }

  @Override
  protected double getLeftDistance() {
    // if (RobotBase.isSimulation()) {
    //   return driveSim.getLeftPositionMeters();
    // }

    return leftOneEncoder.getPosition() * ROTATIONS_TO_METERS;
  }

  @Override
  protected double getRightDistance() {
    // if (RobotBase.isSimulation()) {
    //   return driveSim.getRightPositionMeters();
    // }

    return -rightOneEncoder.getPosition() * ROTATIONS_TO_METERS;
  }

  @Override
  protected void resetEncoders() {
    leftOneEncoder.setPosition(0);
    rightOneEncoder.setPosition(0);

    if (RobotBase.isSimulation()) {
      leftEncoderPosSim.set(0.0);
      rightEncoderPosSim.set(0.0);
    }
  }
}
