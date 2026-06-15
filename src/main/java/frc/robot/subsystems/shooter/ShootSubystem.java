package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShootSubystem extends SubsystemBase {
  private final ShootIO io;

  private double targetRPMs;

  private TalonFX shootMotor = new TalonFX(ShootConstants.Shooter.CAN_ID_LEADER);
  private TalonFX shooterFollower = new TalonFX(ShootConstants.Shooter.CAN_ID_FOLLOWER);
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private VoltageOut voltageOut = new VoltageOut(0);

  public ShootSubystem(ShootIO io) {
    this.io = io;
    configMotor(shootMotor);
    configMotor(shooterFollower);
    shooterFollower.setControl(
        new Follower(ShootConstants.Shooter.CAN_ID_LEADER, MotorAlignmentValue.Opposed));

  }

  private void configMotor(TalonFX motor) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = ShootConstants.Conveyor.Motor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {

  }

  public void startConveyorAndKicker() {
    io.setKickerRPS(ShootConstants.Kicker.DEFAULT_VELOCITY);
    io.setConveyorRPS(ShootConstants.Conveyor.DEFAULT_VELOCITY);
  }

  public void startShooter(AngularVelocity velocity) {
    io.setShooterRPS(velocity);
  }

  private static final double voltsPerRPM = 1.05 * 12.0 / 6000.0;

  private double calculateFeedForward(double velocity) {
    return velocity * voltsPerRPM;
  }

  public void setShootSpeed(double speed) {
    shootMotor.setControl(dutyCycleOut.withOutput(speed));
  }

  public void setVoltage(double volts) {
    shootMotor.setControl(voltageOut.withOutput(volts));
  }

  @AutoLogOutput(key = "Shooter/RPS")
  public double getVelocity() {
    return shootMotor.getVelocity().getValue().in(RotationsPerSecond);
  }

  @AutoLogOutput(key = "Shooter/Voltage")
  public double getVoltage() {
    return shootMotor.getMotorVoltage().getValueAsDouble();
  }


  public void setVelocity(double rpm) {
    this.targetRPMs = rpm;
  }

  public void stopAll() {
    io.setConveyorRPS(RotationsPerSecond.of(0));
    io.stopKicker();
    setVoltage(0);
  }


  public Command stop() {
    return runOnce(
        () -> {
          io.setConveyorRPS(RotationsPerSecond.of(0));
          io.stopKicker();
          setVoltage(0);
        });
  }
}
