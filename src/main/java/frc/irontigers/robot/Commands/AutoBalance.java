// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.irontigers.robot.Subsystems.DriveSystem;

public class AutoBalance extends CommandBase {
  private DriveSystem driveSystem;
  private AHRS gyro;
  private double setpoint;
  private double error;
  /** Creates a new AutoBalance. */
  public AutoBalance(DriveSystem driveSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSystem = driveSystem;
    this.gyro = driveSystem.getGyro();
    setpoint = -0.5;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSystem.setGear(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = setpoint - gyro.getRoll();
     double speed = MathUtil.clamp(-error * ( 0.2/ 3.0), -0.2, 0.2);

     if (speed < .07){
      return;
     }

     driveSystem.drive(speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.drive(0, 0);
  }

  // Returns true when the command should end
  @Override
  public boolean isFinished() {
    return Math.abs(error) <= 2;
  }
}
