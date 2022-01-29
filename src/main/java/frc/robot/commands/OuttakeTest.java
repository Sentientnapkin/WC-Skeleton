package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;


public class OuttakeTest extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;

    public OuttakeTest(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
        outtakeSubsystem.setShooterBack(0);
        System.out.println("shooter enabled");
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
