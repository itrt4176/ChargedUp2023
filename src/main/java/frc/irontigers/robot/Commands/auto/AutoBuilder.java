package frc.irontigers.robot.Commands.auto;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.irontigers.robot.Commands.AutoBalance;
import frc.irontigers.robot.Subsystems.Arm;
import frc.irontigers.robot.Subsystems.Claw;
import frc.irontigers.robot.Subsystems.DriveSystem;
import frc.irontigers.robot.utils.SendableRemovableChooser;

public class AutoBuilder {
    private DriveSystem drive;
    private Arm arm;
    private Claw claw;

    private ShuffleboardTab autoTab;
    private ShuffleboardLayout configLayout;

    private GenericEntry placePiece;
    private SendableRemovableChooser<String> startChooser;
    private SendableRemovableChooser<String> endChooser;

    private StartPosition lastStart;

    public AutoBuilder(DriveSystem drive, Arm arm, Claw claw) {
        this.drive = drive;
        this.arm = arm;
        this.claw = claw;

        autoTab = Shuffleboard.getTab("Auto");
        configLayout = autoTab
                .getLayout("Config", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(2, 3)
                .withProperties(Map.of("Label position", "TOP"));

        placePiece = configLayout.add("Place Game Piece", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

        startChooser = new SendableRemovableChooser<>();
        for (StartPosition startPos : StartPosition.values()) {
            startChooser.addOption(startPos.toString(), startPos.name());
        }

        configLayout.add("Start Position", startChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

        endChooser = new SendableRemovableChooser<>();
        configLayout.add("End Position", endChooser);

        lastStart = null;
    }
    
    public void periodic() {
        StartPosition selectedStart = (startChooser.getSelected() != null) ? StartPosition.valueOf(startChooser.getSelected()) : null;

        if (selectedStart == null || selectedStart.equals(lastStart)) {
            return;
        }

        lastStart = selectedStart;

        endChooser.removeAllOptions();
        for (EndPosition endPos : selectedStart.getValidEnds()) {
            endChooser.addOption(endPos.toString(), endPos.name());
        }
    }

    public Command getAutonomousCommand() {
        StartPosition selectedStart = (startChooser.getSelected() != null) ? StartPosition.valueOf(startChooser.getSelected()) : null;
        EndPosition selectedEnd = (endChooser.getSelected() != null) ? EndPosition.valueOf(endChooser.getSelected()) : null;

        if (selectedStart == null || selectedEnd == null) {
            return null;
        }

        StringBuilder pathBuilder = new StringBuilder();
        pathBuilder.append(selectedStart.getPathPrefix());
        pathBuilder.append(selectedEnd.getPathSuffix());

        PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathBuilder.toString(), selectedEnd.getPathConstraints(), true);

        if (placePiece.getBoolean(false)) {
            return new SequentialCommandGroup(
                new InstantCommand(() -> drive.setRobotPosition(trajectory.getInitialPose())),
                new PlaceHigh(arm),
                new WaitCommand(0.25),
                new InstantCommand(claw::open),
                new WaitCommand(0.25),
                new FollowTrajectory(trajectory, drive)
                    .deadlineWith(new RetractAndSwitchArm(arm, claw)),
                new AutoBalance(drive)
            );
        } else {
            return new SequentialCommandGroup(
                new InstantCommand(() -> drive.setRobotPosition(trajectory.getInitialPose())),
                new FollowTrajectory(trajectory, drive),
                new AutoBalance(drive)
            );
        }
    }

    private enum StartPosition {
        NODE_1("Nodes 1", "B1", List.of(EndPosition.LINE_TABLE)),
        NODE_2("Nodes 2", "B2", List.of(EndPosition.LINE_TABLE)),
        NODE_3("Nodes 3", "B3", List.of(EndPosition.LINE_TABLE)),
        NODE_4("Nodes 4", "B4", List.of(EndPosition.CHARGE_STATION, EndPosition.LINE_TABLE)),
        NODE_5("Nodes 5", "B5", List.of(EndPosition.CHARGE_STATION, EndPosition.LINE_LOAD, EndPosition.LINE_TABLE)),
        NODE_6("Nodes 6", "B6", List.of(EndPosition.CHARGE_STATION, EndPosition.LINE_LOAD)),
        NODE_7("Nodes 7", "B7", List.of(EndPosition.LINE_LOAD)),
        NODE_8("Nodes 8", "B8", List.of(EndPosition.LINE_TABLE));

        private final String repr;
        private final String pathPrefix;
        private final List<EndPosition> validEnds;

        private StartPosition(String repr, String pathPrefix, List<EndPosition> validEnds) {
            this.repr = repr;
            this.pathPrefix = pathPrefix;
            this.validEnds = validEnds;
        }

        /**
         * @return the pathPrefix
         */
        public String getPathPrefix() {
            return pathPrefix;
        }

        /**
         * @return the validEnds
         */
        public List<EndPosition> getValidEnds() {
            return validEnds;
        }

        @Override
        public String toString() {
            return repr;
        }
    }

    private enum EndPosition {
        CHARGE_STATION("Charge Station", "_CS", new PathConstraints(2.75, 1.5)),
        LINE_LOAD("Cross Line (Loading Side)", "_L_Load", new PathConstraints(3.0, 2.0)),
        LINE_TABLE("Cross Line (Staging Side)", "_L_Table", new PathConstraints(3.0, 2.0));

        private final String repr;
        private final String pathSuffix;
        private final PathConstraints pathConstraints;

        private EndPosition(String repr, String pathSuffix, PathConstraints pathConstraints) {
            this.repr = repr;
            this.pathSuffix = pathSuffix;
            this.pathConstraints = pathConstraints;
        }

        /**
         * @return the pathSuffix
         */
        public String getPathSuffix() {
            return pathSuffix;
        }

        /**
         * @return the pathConstraints
         */
        public PathConstraints getPathConstraints() {
            return pathConstraints;
        }

        @Override
        public String toString() {
            return repr;
        }
    }

}
