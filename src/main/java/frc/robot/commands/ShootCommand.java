package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShootSubystem;

public class ShootCommand extends Command {
  private final ShootSubystem shootSubystem;
  private final Drive driveSubsystem;
  private final PIDController aimPidController;

  private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  // Aiming at tag 24
  private final Pose2d targetPose = fieldLayout.getTagPose(24).get().toPose2d();

  public ShootCommand(ShootSubystem shootSubystem, Drive driveSubsystem) {
    this.shootSubystem = shootSubystem;
    this.driveSubsystem = driveSubsystem;
    addRequirements(shootSubystem, driveSubsystem);
    aimPidController = new PIDController(4, 0, .1);
    aimPidController.enableContinuousInput(-Math.PI, Math.PI);
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

  private double calculateAimSpeed() {
    var currentPose = driveSubsystem.getPose();

    Rotation2d toTarget =
      new Rotation2d(
            Math.atan2(
                targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX()));

    Rotation2d currentRot = currentPose.getRotation();

    double error = toTarget.minus(currentRot).getRadians();

    double output = aimPidController.calculate(error);

    return output;
  }
}
