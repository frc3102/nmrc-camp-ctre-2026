package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShootSubystem;

public class ShootCommand extends Command {
  private final ShootSubystem shootSubystem;

  public ShootCommand(ShootSubystem shootSubystem) {
    this.shootSubystem = shootSubystem;
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return super.isFinished();
  }
}
