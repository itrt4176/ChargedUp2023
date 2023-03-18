// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.irontigers.robot.Commands.AutoBalance;
import frc.irontigers.robot.Subsystems.Arm;
import frc.irontigers.robot.Subsystems.Claw;
import frc.irontigers.robot.Subsystems.DriveSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeToChargeStation extends SequentialCommandGroup {
  /** Creates a new ConeToChargeStation. */
  public ConeToChargeStation(DriveSystem driveSystem, Arm arm, Claw claw, String path) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory autoTrajectory = PathPlanner.loadPath(path, 2.0, 0.63, true);

    addCommands(
        new InstantCommand(() -> driveSystem.setRobotPosition(autoTrajectory.getInitialPose())),
        new PlaceHigh(arm),
        new WaitCommand(0.25),
        new InstantCommand(() -> claw.open()),
        new WaitCommand(0.25),
        new FollowTrajectory(autoTrajectory, driveSystem).deadlineWith(new RetractAndSwitchArm(arm, claw)),
        new AutoBalance(driveSystem)
    );
  }
}
