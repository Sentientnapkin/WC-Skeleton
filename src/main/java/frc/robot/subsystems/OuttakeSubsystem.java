package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class OuttakeSubsystem extends SubsystemBase {
    // Setup motors, pid controller, and booleans
    private final TalonFX shooterMotorFront = new TalonFX(7);
    private final TalonFX shooterMotorBack = new TalonFX(8);
    private final CANSparkMax hoodAngleMotor = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final DutyCycleEncoder hoodAngleEncoder = new DutyCycleEncoder(9);

    PIDController hoodAnglePID;
    PIDController frontShooterPID;
    PIDController backShooterPID;

    public boolean shooterRunning = false;
    private double currentFrontShooterPower = 0.0;
    private double currentBackShooterPower = 0.0;
    private double currentHoodAngle = 10.0;


    public OuttakeSubsystem() {
        hoodAngleEncoder.reset();
        hoodAngleEncoder.setDistancePerRotation(360*48/50.0);

        shooterMotorFront.setInverted(true);
        shooterMotorBack.setInverted(false);
        hoodAngleMotor.setInverted(false);

        hoodAnglePID = new PIDController(0.0075, 0.04 ,0);
        frontShooterPID = new PIDController(0, 0 ,0);
        backShooterPID = new PIDController(0, 0 ,0);

        hoodAnglePID.setSetpoint(10.0);
    }

     public void setShooterPower(double power) { // Enables both wheels
        setShooterFront(power);
        setShooterBack(power);
        shooterRunning = true;
    }

    private void setShooterFront(double power) { // Enables front wheels
        shooterMotorFront.set(ControlMode.PercentOutput, power);
    }

    private void setShooterBack(double power) { // Enables back wheels
        shooterMotorBack.set(ControlMode.PercentOutput, power);
    }

    public void stopShooter() { // Disables shooter
        setShooterPower(0);
        shooterRunning = false;
    }

    public void setHoodAngle(double angle) { if (angle>45.0 || angle<10.0) return; this.currentHoodAngle = angle; hoodAnglePID.setSetpoint(angle); }

    public double getHoodAngle() { return Math.abs(hoodAngleEncoder.getDistance() - 256.0); } //DEGREES

    public double getTargetedHoodAngle() { return currentHoodAngle; }

    public void disable() {
        stopShooter();
        //stopHood();
        //stopTurret();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("hoodAngle", getHoodAngle());

        if (shooterRunning) {
            double power = hoodAnglePID.calculate(getHoodAngle());
            hoodAngleMotor.set(power > 0.31 ? 0.3 : power);
        }
    }
}

