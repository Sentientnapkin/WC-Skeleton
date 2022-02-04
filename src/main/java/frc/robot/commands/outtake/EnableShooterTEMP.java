package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;


public class EnableShooterTEMP extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;

    public EnableShooterTEMP(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() { outtakeSubsystem.setShooterPower(0.1); }

    @Override
    public boolean isFinished() { return true; }
}
