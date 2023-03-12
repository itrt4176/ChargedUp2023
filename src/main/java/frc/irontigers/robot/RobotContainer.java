// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot;

import java.time.Instant;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.irontigers.robot.Commands.ArmManualLengthAdjustment;
import frc.irontigers.robot.Commands.AutoArmExtend;
import frc.irontigers.robot.Commands.AutoBalance;
import frc.irontigers.robot.Commands.MoveArmToAngle;
import frc.irontigers.robot.Commands.FollowTrajectory;
import frc.irontigers.robot.Commands.AutoSimpleDrive;
import frc.irontigers.robot.Commands.AutoSimpleReverse;
import frc.irontigers.robot.Subsystems.Arm;
import frc.irontigers.robot.Subsystems.Claw;
import frc.irontigers.robot.Subsystems.DriveSystem;
import frc.tigerlib.XboxControllerIT;
import frc.tigerlib.command.DifferentialJoystickDrive;
import frc.tigerlib.command.ToggleInversionCommand;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here..

  private final XboxControllerIT mainController = new XboxControllerIT(0);
  private final XboxControllerIT clawController = new XboxControllerIT(1);
  
  private final DriveSystem driveSystem = new DriveSystem();
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();

  private final DifferentialJoystickDrive joystickDrive = new DifferentialJoystickDrive(driveSystem, mainController);
  private final ToggleInversionCommand toggleInversion = new ToggleInversionCommand(driveSystem);

  private final Trigger gearShiftUp = mainController.rightBumper();
  private final Trigger gearShiftDown = mainController.leftBumper();

  private final ArmManualLengthAdjustment armLengthAdjustment = new ArmManualLengthAdjustment(arm, mainController);
  private final MoveArmToAngle armSetAngle190 = new MoveArmToAngle(arm, 190);
  private final MoveArmToAngle armSetAngle205 = new MoveArmToAngle(arm, 205);

  // private final AutoArmExtend autoFullRetract = new AutoArmExtend(arm, 0);
  // private final AutoArmExtend autoHalfExtend = new AutoArmExtend(arm, 23/2.0);
  // private final AutoArmExtend autoFullExtend = new AutoArmExtend(arm, 23);

  private final Trigger toggleInvertButton = mainController.b();

  private final Trigger armRotationForward = mainController.y();
  private final Trigger armRotationBackward = mainController.a();
 
  private final Trigger  armSet190 = mainController.povLeft();
  private final Trigger armSet205 = mainController.povRight();

  // private final Trigger fullRetract = mainController.povLeft();
  // private final Trigger halfExtend = mainController.povUp();
  // private final Trigger fullExtend = mainController.povRight();

  private final Trigger clawIn = mainController.povUp();
  private final Trigger clawOut = mainController.povDown();

  private final SendableChooser<String> autoPath = new SendableChooser<>();

 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button binding.
    configureButtonBindings();
    mainController.setDeadzone(.2);
    driveSystem.setDefaultCommand(joystickDrive);
    arm.setDefaultCommand(armLengthAdjustment);
    //Maybe adjust once arm rotation is coded.
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxControllerIT}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    gearShiftUp.onTrue(new InstantCommand(() -> driveSystem.shiftUp()));
    gearShiftDown.onTrue(new InstantCommand(() -> driveSystem.shiftDown()));

    toggleInvertButton.onTrue(toggleInversion);

    armRotationForward.onTrue(new InstantCommand(() -> arm.setRotationSpeed(.15)));
    armRotationForward.onFalse(new InstantCommand(() -> arm.setRotationSpeed(0)));

    armRotationBackward.onTrue(new InstantCommand(() -> arm.setRotationSpeed(-.15)));
    armRotationBackward.onFalse(new InstantCommand(() -> arm.setRotationSpeed(0)));
    // armStopRotation.onTrue(new InstantCommand(() -> arm.setRotationSpeed(0.0)));

    armSet190.onTrue(armSetAngle190);
    armSet205.onTrue(armSetAngle205);

    // clawIn.whileTrue(new StartEndCommand(
    //   () -> claw.setClawOneSpeed(0.75), 
    //   () -> claw.setClawOneSpeed(0)));
    // clawIn.onFalse(new InstantCommand(() -> claw.setClawOneSpeed(0)));

    clawIn.onTrue(new InstantCommand(() -> claw.open()));
    clawOut.onTrue(new InstantCommand(() -> claw.close()));

    // clawOut.whileTrue(new StartEndCommand(
    //   () -> claw.setClawOneSpeed(-0.75), 
    //   () -> claw.setClawOneSpeed(0)));
    // // clawOut.onFalse(new InstantCommand(() -> claw.setClawOneSpeed(0)));

    // fullRetract.onTrue(autoFullRetract);
    // halfExtend.onTrue(autoHalfExtend);
    // fullExtend.onTrue(autoFullExtend);

    autoPath.addOption("Simple Auto", "B4_CS");
    autoPath.addOption("Super Auto", "SuperAuto");
    SmartDashboard.putData("Auto Path", autoPath);
    SmartDashboard.putData("BALANCE!", new AutoBalance(driveSystem));
  }
 
 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String path = autoPath.getSelected();

    PathPlannerTrajectory autoTrajectory = PathPlanner.loadPath(path, 2.0, 0.63, true);

    ParallelCommandGroup angleArmExtending = new ParallelCommandGroup(
        new MoveArmToAngle(arm, 195),
      new SequentialCommandGroup(
            new WaitUntilCommand(() -> arm.getArmDegrees() >= 118.5),
            new AutoArmExtend(arm, 23.6)
      )
    );

    ParallelDeadlineGroup driveRetract = new FollowTrajectory(autoTrajectory, driveSystem).deadlineWith(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
                new WaitUntilCommand(() -> arm.getArmExtensionPosition() <= 23.6 - 12.0),
          new InstantCommand(claw::close)
        ),
        new AutoArmExtend(arm, 0),
        new MoveArmToAngle(arm, 2.5)
      )
    );

    return new SequentialCommandGroup(
        new InstantCommand(() -> driveSystem.setRobotPosition(autoTrajectory.getInitialPose())),
        angleArmExtending,
        new WaitCommand(0.25),
        new InstantCommand(() -> claw.open()),
        new WaitCommand(0.25),
        driveRetract,
        new AutoBalance(driveSystem)
    );
    // return drive;

  }
}
