// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.irontigers.robot.Subsystems.Intake;

public class SmartIntakeControl extends CommandBase {
  private final Intake intake;
  private Debouncer currentDebouncer;
  private Debouncer velocityDebouncer;
  private LinearFilter currentFilter;
  private LinearFilter velocityFilter;

  /** Creates a new SmartIntake. */
  public SmartIntakeControl(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;

    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeSpeed(-0.125);

    // currentDebouncer = new Debouncer(0.1);
    // velocityDebouncer = new Debouncer(0.1);

    
    SmartDashboard.putBoolean("Intake Spinning", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Intake Spinning", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Intake Spinning", false);
    intake.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getCombinedRPM() > -10 && intake.getCombinedSupplyCurrent() > 0.55;
  }
}
