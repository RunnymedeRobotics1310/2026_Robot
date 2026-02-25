package frc.robot.commands.auto.config;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swerve.DriveToTowerCommand;
import frc.robot.commands.swerve.FaceHubCommand;
import frc.robot.commands.swerve.SetAllianceGyroCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;
import java.util.ArrayList;
import java.util.List;

public class AutoCommandFactory {

    private final SwerveSubsystem swerve;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final LimelightVisionSubsystem vision;

    public AutoCommandFactory(
            SwerveSubsystem swerve,
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            LimelightVisionSubsystem vision) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;
        this.vision = vision;
    }

    public Command buildAutoCommand(AutoConfig config, double delay) {
        List<Command> commands = new ArrayList<>();

        if (config == null || config.steps == null) {
            System.out.println("AutoCommandFactory: Invalid config");
            return new WaitCommand(0);
        }

        if (delay > 0) {
            commands.add(new WaitCommand(delay));
        }

        commands.add(new SetAllianceGyroCommand(swerve, config.startingHeadingDegrees));

        for (AutoStep step : config.steps) {
            if (step == null) {
                continue;
            }
            Command cmd = buildStep(step);
            if (cmd != null) {
                commands.add(cmd);
            }
        }

        return new SequentialCommandGroup(commands.toArray(new Command[0]));
    }

    private Command buildStep(AutoStep step) {
        switch (step.type) {
            case drive:
                return buildDriveCommand(step);
            case set_pose:
                return new ConfigSetPoseCommand(
                        swerve,
                        step.xMetres,
                        step.yMetres,
                        step.headingDegrees);
            case rotate:
                return new ConfigRotateCommand(swerve, step.headingDegrees, step.timeoutSeconds);
            case shooter:
                return new ConfigShooterCommand(
                        shooter,
                        step.action,
                        step.rpm,
                        step.hoodPosition,
                        step.kickerSpeed,
                        step.kickerDelaySeconds,
                        step.durationSeconds);
            case intake:
                return new ConfigIntakeCommand(intake, step.intakeAction, step.speed, step.durationSeconds);
            case delay:
                return new WaitCommand(step.durationSeconds);
            case parallel:
                return buildParallelGroup(step);
            case face_target:
                return buildFaceTargetCommand(step);
            case vision_approach_tag:
                return new DriveToTowerCommand(swerve, vision, step.rightSide)
                        .withTimeout(step.timeoutSeconds);
            case hold:
                return new ConfigHoldDriveCommand(swerve, step.durationSeconds);
            default:
                System.out.println("AutoCommandFactory: Unknown step type: " + step.type);
                return null;
        }
    }

    private Command buildFaceTargetCommand(AutoStep step) {
        Command cmd;
        if (step.target == AutoStep.FaceTargetType.hub) {
            cmd = new FaceHubCommand(swerve);
        } else {
            cmd = new ConfigFaceFieldPointCommand(
                    swerve,
                    step.targetXMetres,
                    step.targetYMetres,
                    step.headingToleranceDegrees);
        }
        return cmd.withTimeout(step.timeoutSeconds);
    }

    private Command buildDriveCommand(AutoStep step) {
        if (step.mode == AutoStep.DriveMode.distance) {
            return new ConfigDriveDistanceCommand(
                    swerve,
                    step.direction,
                    step.speedMPS,
                    step.distanceMetres,
                    step.headingDegrees,
                    step.timeoutSeconds);
        } else if (step.mode == AutoStep.DriveMode.time) {
            return new ConfigDriveTimedCommand(
                    swerve,
                    step.direction,
                    step.speedMPS,
                    step.durationSeconds,
                    step.headingDegrees);
        } else if (step.mode == AutoStep.DriveMode.velocity) {
            return new ConfigDriveVelocityCommand(
                    swerve,
                    step.frame,
                    step.vxMPS,
                    step.vyMPS,
                    step.headingDegrees,
                    step.durationSeconds);
        } else {
            return new ConfigDriveToPoseCommand(
                    swerve,
                    step.xMetres,
                    step.yMetres,
                    step.headingDegrees,
                    step.speedMPS,
                    step.positionToleranceMetres,
                    step.headingToleranceDegrees,
                    step.timeoutSeconds);
        }
    }

    private Command buildParallelGroup(AutoStep step) {
        if (step.commands == null || step.commands.isEmpty()) {
            System.out.println("AutoCommandFactory: Parallel group has no commands");
            return new WaitCommand(0);
        }

        List<Command> children = new ArrayList<>();
        for (AutoStep child : step.commands) {
            Command cmd = buildStep(child);
            if (cmd != null) {
                children.add(cmd);
            }
        }

        if (children.isEmpty()) {
            return new WaitCommand(0);
        }

        Command[] cmds = children.toArray(new Command[0]);
        if (step.endCondition == AutoStep.ParallelEndCondition.first) {
            return new ParallelRaceGroup(cmds);
        } else if (step.endCondition == AutoStep.ParallelEndCondition.deadline) {
            int deadlineIndex = Math.max(0, Math.min(step.deadlineIndex, children.size() - 1));
            Command deadline = children.remove(deadlineIndex);
            return new ParallelDeadlineGroup(deadline, children.toArray(new Command[0]));
        } else {
            return new ParallelCommandGroup(cmds);
        }
    }
}
