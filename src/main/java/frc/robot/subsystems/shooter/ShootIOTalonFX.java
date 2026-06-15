package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShootIOTalonFX implements ShootIO {
  private TalonFX conveyor = new TalonFX(ShootConstants.Conveyor.CAN_ID);
  private TalonFX kicker = new TalonFX(ShootConstants.Kicker.CAN_ID);

  // Conveyor Signals
  private StatusSignal<Voltage> conveyorVolts = conveyor.getMotorVoltage();
  private StatusSignal<Current> conveyorSupplyAmps = conveyor.getSupplyCurrent();
  private StatusSignal<AngularVelocity> conveyorVelocity = conveyor.getVelocity();
  private StatusSignal<Temperature> conveyorTemp = conveyor.getDeviceTemp();

  // Kicker Signals
  private StatusSignal<Voltage> kickerVolts = kicker.getMotorVoltage();
  private StatusSignal<Current> kickerSupplyAmps = kicker.getSupplyCurrent();
  private StatusSignal<AngularVelocity> kickerVelocity = kicker.getVelocity();
  private StatusSignal<Temperature> kickerTemp = kicker.getDeviceTemp();

  // Control signals
  private MotionMagicVelocityVoltage conveyorVoltage = new MotionMagicVelocityVoltage(0);
  private MotionMagicVelocityVoltage kickerVoltage = new MotionMagicVelocityVoltage(0);
  private NeutralOut neutralOut = new NeutralOut();

  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private VoltageOut voltageOut = new VoltageOut(0);

  public ShootIOTalonFX() {

    configConveyor(conveyor);
    configKicker(kicker);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        conveyorSupplyAmps,
        conveyorVolts,
        conveyorTemp,
        conveyorVelocity,
        kickerSupplyAmps,
        kickerVolts,
        kickerTemp,
        kickerVelocity);
    ParentDevice.optimizeBusUtilizationForAll(conveyor, kicker);
  }

  private void configConveyor(TalonFX motor) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = ShootConstants.Conveyor.Motor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = ShootConstants.Conveyor.Motor.KP;
    config.Slot0.kI = ShootConstants.Conveyor.Motor.KI;
    config.Slot0.kD = ShootConstants.Conveyor.Motor.KD;
    config.Slot0.kS = ShootConstants.Conveyor.Motor.KS;
    config.Slot0.kG = 0;
    config.Slot0.kV = ShootConstants.Conveyor.Motor.KV;
    config.Slot0.kA = ShootConstants.Conveyor.Motor.KA;

    config.MotionMagic.MotionMagicAcceleration = ShootConstants.Conveyor.Motor.MM_A;
    config.MotionMagic.MotionMagicJerk = ShootConstants.Conveyor.Motor.MM_JERK;
    motor.getConfigurator().apply(config);
  }

  private void configKicker(TalonFX motor) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = ShootConstants.Kicker.Motor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = ShootConstants.Kicker.Motor.KP;
    config.Slot0.kI = ShootConstants.Kicker.Motor.KI;
    config.Slot0.kD = ShootConstants.Kicker.Motor.KD;
    config.Slot0.kS = ShootConstants.Kicker.Motor.KS;
    config.Slot0.kG = 0;
    config.Slot0.kV = ShootConstants.Kicker.Motor.KV;
    config.Slot0.kA = ShootConstants.Kicker.Motor.KA;

    config.MotionMagic.MotionMagicAcceleration = ShootConstants.Kicker.Motor.MM_A;
    config.MotionMagic.MotionMagicJerk = ShootConstants.Kicker.Motor.MM_JERK;
    motor.getConfigurator().apply(config);
  }

  private void configShooterMotor(TalonFX motor) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = ShootConstants.Conveyor.Motor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setShooterRPS(AngularVelocity velocity) {
    // TODO
  }

  @Override
  public void setConveyorRPS(AngularVelocity velocity) {
    conveyor.setControl(conveyorVoltage.withVelocity(velocity));
  }

  @Override
  public void setKickerRPS(AngularVelocity velocity) {
    kicker.setControl(kickerVoltage.withVelocity(velocity));
  }

  public void stopKicker() {
    kicker.setControl(neutralOut);
  }

  @Override
  public boolean isAtSpeed(AngularVelocity velocity) {
    return true;
  }

  public void stopShooter() {
    // TODO
  }

  @Override
  public void updateInputs(ShootIOInputs inputs) {
    boolean conveyorConnected =
        BaseStatusSignal.refreshAll(
                conveyorSupplyAmps, conveyorVolts, conveyorTemp, conveyorVelocity)
            .isOK();
    boolean kickerConnected =
        BaseStatusSignal.refreshAll(kickerSupplyAmps, kickerVolts, kickerTemp, kickerVelocity)
            .isOK();

    inputs.conveyorConnected = conveyorConnected;
    inputs.kickerConnected = kickerConnected;
    inputs.conveyorVolts = conveyorVolts.getValue();
    inputs.kickerVolts = kickerVolts.getValue();
    inputs.conveyorAmps = conveyorSupplyAmps.getValue();
    inputs.kickerAmps = kickerSupplyAmps.getValue();
    inputs.conveyorVelocity = conveyorVelocity.getValue();
    inputs.kickerVelocity = kickerVelocity.getValue();
  }
}
