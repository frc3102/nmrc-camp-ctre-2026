package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShootSubystem;

public class TuningShot extends Command {
  private long rps = 50;
  private ShootSubystem superstructure;
  private AngularVelocity targetVelocity;
  private boolean atSpeed = false;

  public TuningShot(ShootSubystem superstructure) {
    addRequirements(superstructure);
    this.superstructure = superstructure;
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.stopAll();
    atSpeed = false;
  }

  @Override
  public void execute() {
    if (!atSpeed) {
      if (superstructure.isAtSpeed(targetVelocity)) {
        atSpeed = true;
        superstructure.startConveyorAndKicker();
      }
    }
  }

  @Override
  public void initialize() {
    targetVelocity = RotationsPerSecond.of(rps);
    superstructure.startShooter(targetVelocity);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addIntegerProperty("ShooterTuning/RPS", () -> this.rps, (n) -> this.rps = n);
  }
}
