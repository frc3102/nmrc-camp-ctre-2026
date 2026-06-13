package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.SuperstructureIOInputsAutoLogged;
import frc.robot.util.ShootingCalculator;
import org.littletonrobotics.junction.Logger;

public class ShootSubystem extends SubsystemBase {
  private final ShootIO io;
  private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

  private double targetRPMs;

  public ShootSubystem(ShootIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure", inputs);
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

  public void setSpeed(double speed) {
    io.setDutyCycle(speed);
  }

  public void setVoltage(double volts) {
    io.setShooterVoltage(volts);
  }

  public double getVelocity() {
    return inputs.shooterVelocity.in(RotationsPerSecond);
  }

  public void setVelocity(double rpm) {
    this.targetRPMs = rpm;
  }

  public void stopAll() {
    io.setConveyorRPS(RotationsPerSecond.of(0));
    io.stopKicker();
    io.stopShooter();
  }

  public boolean isAtSpeed(AngularVelocity velocity) {
    return io.isAtSpeed(velocity);
  }

  public Command shootAtSpeed(AngularVelocity speed) {
    return runOnce(
        () -> {
          io.setShooterRPS(speed);
          io.setKickerRPS(ShootConstants.Kicker.DEFAULT_VELOCITY);
          io.setConveyorRPS(ShootConstants.Conveyor.DEFAULT_VELOCITY);
        });
  }

  public Command shootAtHub() {
    return run(
        () -> {
          var calc = ShootingCalculator.getInstance();
          var target = calc.getHub();
          var delta = calc.update(target);
          io.setShooterRPS(ShootingCalculator.rpsForDistance(delta.distance()));
          io.setKickerRPS(ShootConstants.Kicker.DEFAULT_VELOCITY);
          io.setConveyorRPS(ShootConstants.Conveyor.DEFAULT_VELOCITY);
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          io.setConveyorRPS(RotationsPerSecond.of(0));
          io.stopKicker();
          io.stopShooter();
        });
  }
}
