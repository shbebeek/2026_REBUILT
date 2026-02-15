package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Superstructure extends SubsystemBase{
    public final IntakeSubsystem intake;
    public final HopperSubsystem hopper;
    public final FeederSubsystem feeder;
    public final TurretSubsystem turret;
    public final ShooterSubsystem shooter;
    public final ClimberSubsystem climber;

    // triggers for readiness checks
    private final Trigger isShooterAtSpeed;
    private final Trigger isTurretAtAngle;
    private final Trigger isReadyToShoot;

    private AngularVelocity targetShooterSpeed = RPM.of(0);
    private Angle targetTurretAngle = Degrees.of(0);

    // default aim point is red hub
    private Translation3d aimPoint = Constants.AimPoints.RED_HUB.value;

    public Superstructure(IntakeSubsystem intake, HopperSubsystem hopper, FeederSubsystem feeder,
        TurretSubsystem turret, ShooterSubsystem shooter, ClimberSubsystem climber){
        this.intake = intake;
        this.hopper = hopper;
        this.feeder = feeder;
        this.turret = turret;
        this.shooter = shooter;
        this.climber = climber;

        // create triggers for checking if mechs at targets
        this.isShooterAtSpeed = new Trigger(
            () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM))
            < Constants.ShooterConstants.SHOOTER_TOLERANCE.in(RPM));
        
        this.isTurretAtAngle = new Trigger(
            () -> Math.abs(turret.getRawAngle().in(Degrees) - targetTurretAngle.in(Degrees))
            < Constants.TurretConstants.TURRET_TOLERANCE.in(Degrees));
        
        this.isReadyToShoot = isShooterAtSpeed.and(isTurretAtAngle);
    }
    
}
