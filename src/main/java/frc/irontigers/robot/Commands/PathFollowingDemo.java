// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot.Commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.irontigers.robot.Subsystems.DriveSystem;
import static frc.irontigers.robot.Constants.DriveVals.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathFollowingDemo extends SequentialCommandGroup {
  /** Creates a new PathFollowingDemo. */
  public PathFollowingDemo(PathPlannerTrajectory trajectory, DriveSystem driveSys) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> driveSys.setRobotPosition(trajectory.getInitialPose())),
        new PPRamseteCommand(
          trajectory, 
          driveSys::getRobotPosition, 
          new RamseteController(), 
          new SimpleMotorFeedforward(S, V, A), 
          driveSys.getKinematics(), 
          driveSys::getWheelSpeeds,
          new PIDController(LEFT_P, LEFT_I, LEFT_D),
          new PIDController(RIGHT_P, RIGHT_I, RIGHT_D),
          driveSys::voltageDrive,
          false,
          driveSys
        ),
        new InstantCommand(() -> driveSys.voltageDrive(0, 0))
    );
  }
}
