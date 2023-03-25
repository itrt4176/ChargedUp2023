// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.irontigers.robot;

import java.time.Instant;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.irontigers.robot.Commands.ArmManualLengthAdjustment;
import frc.irontigers.robot.Commands.AutoArmExtend;
import frc.irontigers.robot.Commands.AutoBalance;
import frc.irontigers.robot.Commands.MoveArmToAngle;
import frc.irontigers.robot.Commands.SmartIntakeControl;
import frc.irontigers.robot.Commands.auto.AutoBuilder;
import frc.irontigers.robot.Commands.auto.ConeToChargeStation;
import frc.irontigers.robot.Commands.auto.FollowTrajectory;
import frc.irontigers.robot.Commands.auto.PlaceHigh;
import frc.irontigers.robot.Commands.AutoSimpleDrive;
import frc.irontigers.robot.Commands.AutoSimpleReverse;
import frc.irontigers.robot.Commands.CommandJoystickDrive;
import frc.irontigers.robot.Commands.HomeArm;
import frc.irontigers.robot.Commands.ManualArmRotation;
import frc.irontigers.robot.Subsystems.Arm;
import frc.irontigers.robot.Subsystems.Claw;
import frc.irontigers.robot.Subsystems.DriveSystem;
import frc.irontigers.robot.Subsystems.Intake;
import frc.tigerlib.XboxControllerIT;
import frc.tigerlib.command.DifferentialJoystickDrive;
import frc.tigerlib.command.ToggleInversionCommand;
import frc.irontigers.robot.Subsystems.Arm.*;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here..

  private final XboxControllerIT mainController = new XboxControllerIT(0);
  private final CommandJoystick forwardJoystick = new CommandJoystick(2);
  private final CommandJoystick turningJoystick = new CommandJoystick(1);
  // private final XboxControllerIT clawController = new XboxControllerIT(1);
  
  private final DriveSystem driveSystem = new DriveSystem();
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();
  private final Intake intake = new Intake();

  private final DifferentialJoystickDrive joystickDrive = new DifferentialJoystickDrive(driveSystem, mainController);
  private final CommandJoystickDrive commandJoystickDrive = new CommandJoystickDrive(driveSystem, forwardJoystick, turningJoystick);

  private final ManualArmRotation armRotation = new ManualArmRotation(arm, mainController /*rightJoystick*/);

  private final ArmManualLengthAdjustment armLengthAdjustment = new ArmManualLengthAdjustment(arm, mainController);

  private final SequentialCommandGroup homeArm = new SequentialCommandGroup(
    new ParallelCommandGroup(
      new MoveArmToAngle(arm, 15),
      new AutoArmExtend(arm, 0)
    ),
    new HomeArm(arm)
  );

  private final AutoArmExtend autoFullExtend = new AutoArmExtend(arm, 23.5);
  private final AutoArmExtend autoHalfExtend = new AutoArmExtend(arm, 11);
  private final AutoArmExtend autoFullRetract = new AutoArmExtend(arm, 0);

  private final Command intakeControl = new RepeatCommand(new SmartIntakeControl(intake)
    .andThen(new WaitUntilCommand(() -> {
        return intake.getCombinedRPM() >= 47.5;
      })
  ));

  private final Trigger armHomingButton = mainController.povLeft();
  private final Trigger armSetLowPole = mainController.povUp();
  private final Trigger armSetTopPole = mainController.povRight();
  
  private final Trigger armLock = turningJoystick.button(8);

  private final Trigger armRetractButton = turningJoystick.trigger();
  private final Trigger armExtendButton = forwardJoystick.trigger();

  private final Trigger halfExtend = mainController.a();
  private final Trigger fullRetract = mainController.b();
  private final Trigger fullExtend = mainController.x();
  private final Trigger toggleClaw = mainController.y();

  private final Trigger gearShiftUp = mainController.rightBumper();
  private final Trigger gearShiftDown = mainController.leftBumper();

  private final Trigger armRotateUp = turningJoystick.button(5);
  private final Trigger armRotateDown = forwardJoystick.button(4);

  private final Trigger intakeIn = turningJoystick.button(2);
  private final Trigger intakeOut = turningJoystick.button(3);

  private final Trigger clawToggle = forwardJoystick.button(3);

  private final Trigger deployTeleop = new Trigger(DriverStation::isTeleopEnabled);

  private final SendableChooser<Command> autoPath = new SendableChooser<>();

  private final AutoBuilder autoBuilder = new AutoBuilder(driveSystem, arm, claw);

 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button binding.
    configureButtonBindings();
    driveSystem.setDefaultCommand(commandJoystickDrive);
    intake.setDefaultCommand(intakeControl);
    // mainController.setDeadzone(.2);
    // driveSystem.setDefaultCommand(joystickDrive);
    // arm.setDefaultCommand(new ParallelCommandGroup(
    //   armLengthAdjustment,
    //   armRotation
    // ));
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

    intakeIn.onTrue(intakeControl);
    // intakeIn.onFalse(new InstantCommand(() -> intake.setIntakeSpeed(0)));

    intakeOut.whileTrue(new StartEndCommand(
        () -> intake.setIntakeSpeed(0.11),
        () -> intake.setIntakeSpeed(0.0),
        intake
    ));
    intakeOut.onFalse(intakeControl);
    // intakeOut.onFalse(new InstantCommand(() -> intake.setIntakeSpeed(0)));

    armRotateUp.onTrue(new InstantCommand(() -> arm.setRotationSpeed(0.7)));
    armRotateUp.onFalse(new InstantCommand(() -> arm.setRotationSpeed(0.0)));

    armRotateDown.onTrue(new InstantCommand(() -> arm.setRotationSpeed(-0.7)));
    armRotateDown.onFalse(new InstantCommand(() -> arm.setRotationSpeed(0.0)));

    // armSetTopPole.onTrue(new SequentialCommandGroup(
    //     new AutoArmExtend(arm, 0),
    //     new MoveArmToAngle(arm, 180)
    // ));

    // armHomingButton.onTrue(homeArm);

    // armSetLowPole.onTrue(new SequentialCommandGroup(
    //   new AutoArmExtend(arm, 0),
    //   new MoveArmToAngle(arm, 190)
    // ));
    // armSetLowPole.onTrue(new SequentialCommandGroup(
    //   new AutoArmExtend(arm, 0),
    //   new MoveArmToAngle(arm, 190)
    // ));

    clawToggle.toggleOnTrue(new StartEndCommand(claw::open, claw::close));

    deployTeleop.onTrue(new InstantCommand(arm::armLockOn));
    armLock.toggleOnTrue(new StartEndCommand(arm::armLockOn, arm::armLockOff));

    fullRetract.onTrue(autoFullRetract);
    halfExtend.onTrue(autoHalfExtend);
    fullExtend.onTrue(autoFullExtend);

    armExtendButton.whileTrue(new StartEndCommand(
        () -> arm.setExtensionSpeed(-0.55), 
        () -> arm.setExtensionSpeed(0)
      ));
    armRetractButton.whileTrue(new StartEndCommand(
        () -> arm.setExtensionSpeed(0.55),
        () -> arm.setExtensionSpeed(0)
      ));

    
  }
 
 

  /**
   * @return the autoBuilder
   */
  public AutoBuilder getAutoBuilder() {
    return autoBuilder;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoPath.getSelected();

  }
}
