package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;


public class ChangeShooterPowerTEMP extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;
    private final double powerToChangeBy;

    public ChangeShooterPowerTEMP(OuttakeSubsystem outtakeSubsystem, double powerToChangeBy) {
        this.outtakeSubsystem = outtakeSubsystem;
        this.powerToChangeBy = powerToChangeBy;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() { outtakeSubsystem.setShooterPower(outtakeSubsystem.getFrontShooterPower() + powerToChangeBy); }

    @Override
    public boolean isFinished() { return true; }
}
