// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.irontigers.robot.Subsystems.Arm;
import frc.tigerlib.XboxControllerIT;

public class MoveArmToAngle extends CommandBase {
  private Arm arm;
  private double armAngle;
  private double setpoint;
  private double error;
  /** Creates a new ArmSetAngle. */
  public MoveArmToAngle(Arm arm, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.armAngle = armAngle;
    this.setpoint = setpoint;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = setpoint - arm.getArmDegrees();
    double speed = MathUtil.clamp(error * (0.7 / 30.0), -0.7, 0.7);

    if (Math.abs(speed) < 0.2) {
      return;
    }

    // if (error >= 30){
    //   speed = 0.7;
    // } else if(error <= -30){
    //   speed = -0.7;
    // }

    arm.setRotationSpeed(speed);
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setRotationSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) <= 0.5;
  }
}
