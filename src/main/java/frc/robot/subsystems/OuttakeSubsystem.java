package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class OuttakeSubsystem extends SubsystemBase {
    // Setup motors, pid controller, and booleans
    private final TalonFX shooterMotorFront = new TalonFX(7);
    private final TalonFX shooterMotorBack = new TalonFX(8);
    private final CANSparkMax hoodAngleMotor = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final Encoder hoodAngleEncoder = new Encoder(8, 9);

    PIDController hoodAnglePID;
    PIDController frontShooterPID;
    PIDController backShooterPID;

    public boolean shooterRunning = false;
    private double currentFrontShooterPower = 0.0;
    private double currentBackShooterPower = 0.0;
    private double currentHoodAngle = 30.0;


    public OuttakeSubsystem() {
        hoodAngleEncoder.reset();
        hoodAngleEncoder.setDistancePerPulse((360*50/48.0)/2048);

        shooterMotorFront.setInverted(false);
        shooterMotorBack.setInverted(false);
        hoodAngleMotor.setInverted(false);

        hoodAnglePID = new PIDController(0.035, 0.12,0.001);
        frontShooterPID = new PIDController(0, 0 ,0);
        backShooterPID = new PIDController(0, 0 ,0);

        hoodAnglePID.setSetpoint(30.0);
    }

     public void setShooterPower(double power) { // Enables both wheels
        if (power<=1.0 && power>=0.0) {
            setShooterFront(power);
            setShooterBack(power);
            shooterRunning = true;
        }
    }

    private void setShooterFront(double power) { shooterMotorFront.set(ControlMode.PercentOutput, power); currentFrontShooterPower = power; }

    private void setShooterBack(double power) { shooterMotorBack.set(ControlMode.PercentOutput, power); currentBackShooterPower = power; }

    public void stopShooter() { // Disables shooter
        setShooterPower(0);
        shooterRunning = false;
    }

    public void setHoodAngle(double angle) { if (angle<=45.0 && angle>=10.0) { this.currentHoodAngle = angle; hoodAnglePID.setSetpoint(angle); }}

    public double getHoodAngle() { return Math.abs(9+hoodAngleEncoder.getDistance()); } //DEGREES

    public double getTargetedHoodAngle() { return currentHoodAngle; }

    public double getFrontShooterPower() { return currentFrontShooterPower; }

    public double getBackShooterPower() { return currentBackShooterPower; }

        public void disable() {
        stopShooter();
        hoodAnglePID.reset();
        //stopHood();
        //stopTurret();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("hoodAngle", getHoodAngle());
        SmartDashboard.putNumber("targeted hoodAngle", currentHoodAngle);
        SmartDashboard.putNumber("shooter power", currentFrontShooterPower);

        if (shooterRunning) {
            double power = hoodAnglePID.calculate(getHoodAngle());
            if (power > 0.5) power = 0.5;
            if (power < -0.5) power = -0.5;
            hoodAngleMotor.set(power);
        }
    }
}

