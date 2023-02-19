// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.irontigers.robot.Subsystems.DriveSystem;

public class AutoSimpleDrive extends CommandBase {
  DriveSystem driveSystem;
    Pose2d currentPos;
    Pose2d destination;
  /** Creates a new AutoSimpleDrive. */
  public AutoSimpleDrive(DriveSystem driveSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSystem = driveSystem;
    addRequirements(this.driveSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPos = driveSystem.getRobotPosition();
    destination = currentPos.plus(new Transform2d(currentPos, new Pose2d(Units.feetToMeters(5), 0, new Rotation2d())));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPos = driveSystem.getRobotPosition();
    driveSystem.drive(.35, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(currentPos.getX()) >= Math.abs(destination.getX());
  }
}
