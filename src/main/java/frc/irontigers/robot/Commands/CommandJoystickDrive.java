// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.tigerlib.subsystem.drive.DifferentialDriveSubsystem;
import frc.irontigers.robot.Subsystems.DriveSystem;
import frc.irontigers.robot.Subsystems.DriveSystem.*;

public class CommandJoystickDrive extends CommandBase {
  private DriveSystem driveSys;
  private CommandJoystick leftJoystick;
  private CommandJoystick rightJoystick;
  /** Creates a new CommandJoystickDrive. */
  public CommandJoystickDrive(DriveSystem driveSubsystem, CommandJoystick leftJoystick, CommandJoystick rightJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSys = driveSubsystem;
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    addRequirements(driveSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSys.twoStickDrive(leftJoystick.getY(), -rightJoystick.getX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
