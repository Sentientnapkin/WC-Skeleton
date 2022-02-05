package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;


public class ChangeHoodAngleTEMP extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;
    double angleToChangeBy;

    public ChangeHoodAngleTEMP(OuttakeSubsystem outtakeSubsystem, double angleToChangeBy) {
        this.outtakeSubsystem = outtakeSubsystem;
        this.angleToChangeBy = angleToChangeBy;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() { /*outtakeSubsystem.setHoodAngle(outtakeSubsystem.getTargetedHoodAngle() + angleToChangeBy);*/ }

    @Override
    public boolean isFinished() { return true; }
}
