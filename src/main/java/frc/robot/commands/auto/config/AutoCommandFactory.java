package frc.robot.commands.auto.config;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swerve.SetAllianceGyroCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;

public class AutoCommandFactory {

    private final SwerveSubsystem swerve;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    public AutoCommandFactory(SwerveSubsystem swerve, ShooterSubsystem shooter, IntakeSubsystem intake) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;
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
            default:
                System.out.println("AutoCommandFactory: Unknown step type: " + step.type);
                return null;
        }
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
        } else {
            return new ConfigDriveTimedCommand(
                    swerve,
                    step.direction,
                    step.speedMPS,
                    step.durationSeconds,
                    step.headingDegrees);
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
        } else {
            return new ParallelCommandGroup(cmds);
        }
    }
}
