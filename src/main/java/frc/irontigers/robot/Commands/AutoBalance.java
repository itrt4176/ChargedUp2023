// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// WITH MUCH GRATITUDE TO TEAM 5924
// Based on https://github.com/Team5924/FRC-2023/blob/dev/src/main/java/org/first5924/frc2023/commands/drive/AutoEngageChargeStation.java

package frc.irontigers.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.irontigers.robot.Subsystems.DriveSystem;
import static frc.irontigers.robot.Constants.AutoVals.*;

import com.kauailabs.navx.frc.AHRS;

public class AutoBalance extends CommandBase {
  private final DriveSystem mDrive;
  private double mStartSettleTimestamp;
  private boolean misWaitingForSettle = false;
  private AHRS gyro;
  private double allowedError;

  /** Creates a new AutoEngageChargeStation. */
  public AutoBalance(DriveSystem drive) {
    mDrive = drive;
    gyro = mDrive.getGyro();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mStartSettleTimestamp = Timer.getFPGATimestamp();
    mDrive.setGear(4);
    allowedError = kAllowedChargeStationErrorDegrees;
    misWaitingForSettle = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (misWaitingForSettle) {
      if (Timer.getFPGATimestamp() >= mStartSettleTimestamp + 1) {
        misWaitingForSettle = false;
      }
    } else {
      if (Math.abs(gyro.getRoll()) > allowedError) {
        mDrive.drive(getAdjustSpeed(gyro.getRoll()), 0);
      } else {
        mDrive.drive(0, 0);
        misWaitingForSettle = true;
        mStartSettleTimestamp = Timer.getFPGATimestamp();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.drive(0, 0);
    mDrive.setGear(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getAdjustSpeed(double angle) {
    // if (angle < kAllowedChargeStationErrorDegrees / 2.0) {
    // return 0;
    // }

    if (angle > 0) {
      return -MathUtil.clamp((0.136 * angle) + 0.06, 0.115, 0.4);
    } else {
      return -MathUtil.clamp((0.136 * angle) - 0.06, -0.4, -0.115);
    }
  }
}
