package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShootIO {
  @AutoLog
  public static class ShootIOInputs {
    public boolean conveyorConnected = false;
    public boolean kickerConnected = false;

    public Voltage conveyorVolts = Volts.of(0);
    public Voltage kickerVolts = Volts.of(0);
    public Current conveyorAmps = Amps.of(0);
    public Current kickerAmps = Amps.of(0);
    public AngularVelocity conveyorVelocity = RotationsPerSecond.of(0);
    public AngularVelocity kickerVelocity = RotationsPerSecond.of(0);

    public Temperature conveyorTemp = Celsius.of(0);
    public Temperature kickerTemp = Celsius.of(0);
  }

  public default void updateInputs(ShootIOInputs inputs) {}

  public default void setConveyorRPS(AngularVelocity velocity) {}

  public default void setKickerRPS(AngularVelocity velocity) {}

  public default void setShooterRPS(AngularVelocity velocity) {}

  public default void setShooterVoltage(double volts) {}

  public default boolean isAtSpeed(AngularVelocity velocity) {
    return true;
  }

  public default void stopKicker() {}

  public default void stopShooter() {}

  public default void setDutyCycle(double speed) {}
}
