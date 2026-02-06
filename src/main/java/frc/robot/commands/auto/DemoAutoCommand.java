package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DemoAutoCommand extends SequentialCommandGroup {


    public DemoAutoCommand(SwerveSubsystem swerve) {

        addCommands(new InstantCommand());
    }

}
