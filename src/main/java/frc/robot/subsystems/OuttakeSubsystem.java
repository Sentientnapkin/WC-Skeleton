package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LinearServo;


public class OuttakeSubsystem extends SubsystemBase {
    // Setup motors, pid controller, and booleans
    private final TalonFX shooterMotorFront = new TalonFX(7);
    private final TalonFX shooterMotorBack = new TalonFX(8);
    private final LinearServo leftHoodAngleServo = new LinearServo(6, 140, 24);
    private final LinearServo rightHoodAngleServo = new LinearServo(7, 140, 24);

    PIDController frontShooterPID;
    PIDController backShooterPID;

    public boolean shooterRunning = false;
    private double currentFrontShooterPower = 0.0;
    private double currentBackShooterPower = 0.0;
    private double currentHoodAngle = 30.0;


    public OuttakeSubsystem() {
        shooterMotorFront.setInverted(false);
        shooterMotorBack.setInverted(false);

        frontShooterPID = new PIDController(0, 0 ,0);
        backShooterPID = new PIDController(0, 0 ,0);
    }

     public void setShooterPower(double power) { // Enables both wheels
        if (power<=1.0 && power>=0.0) {
            setShooterFront(power);
            setShooterBack(power);
            shooterRunning = true;
        }
    }

    public void setHoodAngle(double angle) { if (angle<=45.0 && angle>=9.0) { leftHoodAngleServo.setPosition(140*(angle-9)/36.0); rightHoodAngleServo.setPosition(140*(angle-9)/36.0); }}

    private void setShooterFront(double power) { shooterMotorFront.set(ControlMode.PercentOutput, power); currentFrontShooterPower = power; }

    private void setShooterBack(double power) { shooterMotorBack.set(ControlMode.PercentOutput, power); currentBackShooterPower = power; }

    public double getTargetedHoodAngle() { return currentHoodAngle; }

    public double getFrontShooterPower() { return currentFrontShooterPower; }

    public double getBackShooterPower() { return currentBackShooterPower; }

    public void stopShooter() { // Disables shooter
        setShooterPower(0);
        shooterRunning = false;
    }

    public void stopHood() { leftHoodAngleServo.setSpeed(0); rightHoodAngleServo.setSpeed(0); }

    public void disable() {
        stopShooter();
        //stopTurret();
    }

    @Override
    public void periodic() {
        leftHoodAngleServo.updateCurPos();
        rightHoodAngleServo.updateCurPos();
        setHoodAngle(9);
    }
}

