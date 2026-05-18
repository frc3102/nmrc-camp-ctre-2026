package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SuperstructureIOTalonFX implements SuperstructureIO {
  private TalonFX conveyor = new TalonFX(SuperstructureConstants.Conveyor.CAN_ID);
  private TalonFX kicker = new TalonFX(SuperstructureConstants.Kicker.CAN_ID);
  private TalonFX shooterLeader = new TalonFX(SuperstructureConstants.Shooter.CAN_ID_LEADER);
  private TalonFX shooterFollower = new TalonFX(SuperstructureConstants.Shooter.CAN_ID_FOLLOWER);

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

  // Shooter Leader Signals
  private StatusSignal<Voltage> shooterLeaderVolts = shooterLeader.getMotorVoltage();
  private StatusSignal<Current> shooterLeaderSupplyAmps = shooterLeader.getSupplyCurrent();
  private StatusSignal<AngularVelocity> shooterLeaderVelocity = shooterLeader.getVelocity();
  private StatusSignal<Temperature> shooterLeaderTemp = shooterLeader.getDeviceTemp();

  private AngularVelocity shooterTargetVelocity = RotationsPerSecond.of(0);
  // Shooter Follower Signals
  private StatusSignal<Voltage> shooterFollowerVolts = shooterFollower.getMotorVoltage();
  private StatusSignal<Current> shooterFollowerSupplyAmps = shooterFollower.getSupplyCurrent();
  private StatusSignal<Temperature> shooterFollowerTemp = shooterFollower.getDeviceTemp();

  // Control signals
  private MotionMagicVelocityVoltage conveyorVoltage = new MotionMagicVelocityVoltage(0);
  private MotionMagicVelocityVoltage kickerVoltage = new MotionMagicVelocityVoltage(0);
  private NeutralOut neutralOut = new NeutralOut();

  public SuperstructureIOTalonFX() {

    configConveyor(conveyor);
    configKicker(kicker);
    configShooterMotor(shooterLeader);
    configShooterMotor(shooterFollower);

    shooterFollower.setControl(
        new Follower(SuperstructureConstants.Shooter.CAN_ID_LEADER, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        conveyorSupplyAmps,
        conveyorVolts,
        conveyorTemp,
        conveyorVelocity,
        kickerSupplyAmps,
        kickerVolts,
        kickerTemp,
        kickerVelocity,
        shooterLeaderSupplyAmps,
        shooterLeaderVolts,
        shooterLeaderTemp,
        shooterLeaderVelocity,
        shooterFollowerSupplyAmps,
        shooterFollowerVolts,
        shooterFollowerTemp);
    ParentDevice.optimizeBusUtilizationForAll(conveyor, kicker, shooterLeader, shooterFollower);
  }

  private void configConveyor(TalonFX motor) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = SuperstructureConstants.Conveyor.Motor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = SuperstructureConstants.Conveyor.Motor.KP;
    config.Slot0.kI = SuperstructureConstants.Conveyor.Motor.KI;
    config.Slot0.kD = SuperstructureConstants.Conveyor.Motor.KD;
    config.Slot0.kS = SuperstructureConstants.Conveyor.Motor.KS;
    config.Slot0.kG = 0;
    config.Slot0.kV = SuperstructureConstants.Conveyor.Motor.KV;
    config.Slot0.kA = SuperstructureConstants.Conveyor.Motor.KA;

    config.MotionMagic.MotionMagicAcceleration = SuperstructureConstants.Conveyor.Motor.MM_A;
    config.MotionMagic.MotionMagicJerk = SuperstructureConstants.Conveyor.Motor.MM_JERK;
    motor.getConfigurator().apply(config);
  }

  private void configKicker(TalonFX motor) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = SuperstructureConstants.Kicker.Motor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = SuperstructureConstants.Kicker.Motor.KP;
    config.Slot0.kI = SuperstructureConstants.Kicker.Motor.KI;
    config.Slot0.kD = SuperstructureConstants.Kicker.Motor.KD;
    config.Slot0.kS = SuperstructureConstants.Kicker.Motor.KS;
    config.Slot0.kG = 0;
    config.Slot0.kV = SuperstructureConstants.Kicker.Motor.KV;
    config.Slot0.kA = SuperstructureConstants.Kicker.Motor.KA;

    config.MotionMagic.MotionMagicAcceleration = SuperstructureConstants.Kicker.Motor.MM_A;
    config.MotionMagic.MotionMagicJerk = SuperstructureConstants.Kicker.Motor.MM_JERK;
    motor.getConfigurator().apply(config);
  }

  private void configShooterMotor(TalonFX motor) {
    // TODO
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
  public void updateInputs(SuperstructureIOInputs inputs) {
    boolean conveyorConnected =
        BaseStatusSignal.refreshAll(
                conveyorSupplyAmps, conveyorVolts, conveyorTemp, conveyorVelocity)
            .isOK();
    boolean kickerConnected =
        BaseStatusSignal.refreshAll(kickerSupplyAmps, kickerVolts, kickerTemp, kickerVelocity)
            .isOK();
    boolean shooterLeaderConnected =
        BaseStatusSignal.refreshAll(
                shooterLeaderSupplyAmps,
                shooterLeaderVolts,
                shooterLeaderTemp,
                shooterLeaderVelocity)
            .isOK();
    boolean shooterFollowerConnected =
        BaseStatusSignal.refreshAll(
                shooterFollowerSupplyAmps, shooterFollowerVolts, shooterFollowerTemp)
            .isOK();

    inputs.conveyorConnected = conveyorConnected;
    inputs.kickerConnected = kickerConnected;
    inputs.shooterLeaderConnected = shooterLeaderConnected;
    inputs.shooterFollowerConnected = shooterFollowerConnected;

    inputs.conveyorVolts = conveyorVolts.getValue();
    inputs.kickerVolts = kickerVolts.getValue();
    inputs.shooterLeaderVolts = shooterLeaderVolts.getValue();
    inputs.shooterFollowerVolts = shooterFollowerVolts.getValue();

    inputs.conveyorAmps = conveyorSupplyAmps.getValue();
    inputs.kickerAmps = kickerSupplyAmps.getValue();
    inputs.shooterLeaderAmps = shooterLeaderSupplyAmps.getValue();
    inputs.shooterFollowerAmps = shooterFollowerSupplyAmps.getValue();

    inputs.conveyorVelocity = conveyorVelocity.getValue();
    inputs.kickerVelocity = kickerVelocity.getValue();
    inputs.shooterVelocity = shooterLeaderVelocity.getValue();
    inputs.shooterTargetVelocity = shooterTargetVelocity.copy();
  }
}
