package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    public IntakeSubsystem() {
    }

    public void setSpeed(double speed) {
        System.out.println("SETTING INTAKE SPEED TO: " + speed);
    }

    public void stop() {
        System.out.println("STOPPING INTAKE");
    }

    @Override
    public String toString() {
        return "IntakeSubsystem";
    }
}
