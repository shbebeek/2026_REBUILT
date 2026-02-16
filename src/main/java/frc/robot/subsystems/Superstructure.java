package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    
    // stops all mechanisms from moving
    public Command stopAllCommand(){
        return Commands.parallel(
            shooter.stop().asProxy(),
            turret.set(0).asProxy(),
            climber.stopMotion().asProxy()).withName("Superstructure.StopAll");
    }

    /**
     * Aims the superstructure to specific targets - used for auto-targeting.
     * @param shooterSpeed Target shooter speed
     * @param turretAngle Target turret angle
     */
    public Command aimCommand(AngularVelocity shooterSpeed, Angle turretAngle){
        return Commands.runOnce(() -> {
            targetShooterSpeed = shooterSpeed;
            targetTurretAngle = turretAngle;
        }).andThen(
            Commands.parallel(
                // shooter.setSpeed(shooterSpeed).asProxy(),
                turret.setAngle(turretAngle).asProxy()
            )
        ).withName("Superstructure.Aim");
    }

    public void setShooterSetpoints(AngularVelocity shooterSpeed, Angle turretAngle){
        targetShooterSpeed = shooterSpeed;
        targetTurretAngle = turretAngle;
    }

    /**
     * Aim superstructure using suppliers - useful for dynamic targeting.
     * @param shooterSpeedSupplier Supplier for target shooter speed
     * @param turretAngleSupplier Supplier for target turret angle
     */
    public Command aimDynamicCommand(
            Supplier<AngularVelocity> shooterSpeedSupplier,
            Supplier<Angle> turretAngleSupplier){
        return Commands.parallel(
            shooter.setSpeedDynamic(shooterSpeedSupplier).asProxy(),
            turret.setAngleDynamic(turretAngleSupplier).asProxy()
        ).withName("Superstructure.AimDynamic");
    }

    // waits until superstructure ready to shoot
    public Command waitUntilReadyCommand(){
         return Commands.waitUntil(isReadyToShoot).withName("Superstructure.WaitUntilReady");
    }

    // aims and waits until ready - combines aim and wait
    public Command aimAndWaitCommand(AngularVelocity shooterSpeed, Angle turretAngle){
        return aimDynamicCommand(() -> shooterSpeed, () -> turretAngle).andThen(waitUntilReadyCommand()).withName("Superstructure.AimAndWait");
    }

    // manual turret control
    public Command setTurretForward(){
        return turret.setAngle(Degrees.of(0)).withName("Superstructure.SetTurretForward");
    }

    public Command setTurretLeft(){
        return turret.setAngle(Degrees.of(45)).withName("Superstructure.SetTurretLeft");
    }

    public Command setTurretRight(){
        return turret.setAngle(Degrees.of(-45)).withName("Superstructure.SetTurretRight");
    }

    public Command rotateTurretLeft(){
        return turret.set(Constants.TurretConstants.MAX_TURRET_SPEED).withName("Superstructure.RotateTurretLeft");
    }

    public Command rotateTurretRight(){
        return turret.set(-Constants.TurretConstants.MAX_TURRET_SPEED).withName("Superstructure.RotateTurretRight");
    }

    // getters for current state
    public AngularVelocity getShooterSpeed(){
        return shooter.getSpeed();
    }

    public Angle getTurretAngle(){
        return turret.getRawAngle();
    }

    public AngularVelocity getTargetShooterSpeed(){
        return targetShooterSpeed;
    }

    public Angle getTargetTurretAngle(){
        return targetTurretAngle;
    }

    public Translation3d getAimPoint(){
        return aimPoint;
    }

    public void setAimPoint(Translation3d newAimPoint){
        this.aimPoint = newAimPoint;
    }

    public Rotation3d getAimRotation3d(){
        return null;
    }
}
