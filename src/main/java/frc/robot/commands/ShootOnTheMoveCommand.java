package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootOnTheMoveCommand extends Command{
    private final SwerveSubsystem drivetrain;
    private final Superstructure superstructure;
    
    private Supplier<Translation3d> aimPointSupplier; // point to aim at
    private AngularVelocity latestShootSpeed;
    private Angle latestTurretAngle;
    private Angle latestHoodAngle;

    public ShootOnTheMoveCommand(SwerveSubsystem drivetrain, Superstructure superstructure,
            Supplier<Translation3d> aimPointSupplier){
        this.drivetrain = drivetrain;
        this.superstructure = superstructure;

        this.aimPointSupplier = aimPointSupplier;
        // right now, when you start another command, auto aim can't start back up again
    }

    @Override
    public void initialize(){
        super.initialize();

        latestHoodAngle = superstructure.getHoodAngle();
        latestTurretAngle = superstructure.getTurretAngle();
        latestShootSpeed = superstructure.getShooterSpeed();

        // TODO: probably should --> when this current command ends, cancel dynamic aim command
        superstructure.aimDynamicCommand(
            () -> {
                return this.latestShootSpeed;
            },
            () -> {
                return this.latestTurretAngle;
            },
            () -> {
                return this.latestHoodAngle;
            }
        ).schedule();
    }

    private double getFlightTime(Distance distanceToTarget){
        // simple linear approximation based on empirical data
        return TIME_OF_FLIGHT_BY_DISTANCE.get(distanceToTarget.in(Meters));
    }

    private AngularVelocity calculateRequiredShooterSpeed(Distance distanceToTarget){
        return RPM.of(SHOOTING_SPEED_BY_DISTANCE.get(distanceToTarget.in(Meters)));
    }

    /* if hood adjustment is added
    private Angle calculateRequiredHoodAngle(Distance distanceToTarget){
        return Degrees.of(HOOD_ANGLE_BY_DISTANCE.get(distanceToTarget.in(Meters)));
    }
    */

    @Override
    public void execute(){
        // calculate trajectory to aimPoint
        var target = aimPointSupplier.get();

        var shooterLocation = drivetrain.getPose3d().getTranslation()
            .plus(superstructure.getShooterPose().getTranslation());
        
        // ignore deltaH for now, range tables will account for it
        // var deltaH = target.getMeasureZ().minus(shooterLocation.getMeasureZ());

        var shooterOnGround = new Translation2d(shooterLocation.getX(), shooterLocation.getY());
        var targetOnGround = new Translation2d(target.getX(), target.getY());

        var distanceToTarget = Meters.of(shooterOnGround.getDistance(targetOnGround));

        // get time of flight --> could try to do this analytically but for now
        // easier and more realistic to use simple linear approximation based on empirical data
        double timeOfFlight = getFlightTime(distanceToTarget);

        // calculate corrective vector based on current velocity multiplied by time of flight
        // if stationary, should be zero. if backing up, ahead of target. if moving forward, behind target
        var updatedPosition = drivetrain.getFieldVelocity().times(timeOfFlight);
        var correctiveVector = new Translation2d(updatedPosition.vxMetersPerSecond, updatedPosition.vyMetersPerSecond).unaryMinus();
        var correctiveVector3d = new Translation3d(correctiveVector.getX(), correctiveVector.getY(), 0);
        
        Logger.recordOutput("FieldSimulation/AimTargetCorrected", new Pose3d(target.plus(correctiveVector3d), Rotation3d.kZero));
        
        var correctedTarget = targetOnGround.plus(correctiveVector);
        var vectorToTarget = drivetrain.getPose().getTranslation().minus(correctedTarget);

        var correctedDistance = Meters.of(vectorToTarget.getNorm());
        var calculatedHeading = vectorToTarget.getAngle()
            .rotateBy(drivetrain.getHeading().unaryMinus())
            .getMeasure();
        
        Logger.recordOutput("ShootOnTheMove/RobotHeading", drivetrain.getHeading());
        Logger.recordOutput("ShootOnTheMove/CalculatedHeading", calculatedHeading);
        Logger.recordOutput("ShootOnTheMove/distanceToTarget", distanceToTarget);

        latestTurretAngle = calculatedHeading;
        latestShootSpeed = calculateRequiredShooterSpeed(correctedDistance);

        // if real hood, add this in
        // else set to current angle
        // latestHoodAngle = calculateRequiredHoodAngle(correctedDistance);
        latestHoodAngle = superstructure.getHoodAngle();

        superstructure.setShooterSetpoints(
            latestShootSpeed,
            latestTurretAngle,
            latestHoodAngle
        );

        System.out.println("Shooting at distance: " + correctedDistance + "requires speed: " + latestShootSpeed + ", hood angle " + latestHoodAngle + ", turret angle " + latestTurretAngle);
    }

    // I'm dumping the constants from the graph in this file instead of Constants.java cus
    // cus this file is scuffed anyways

    // meters, seconds
    private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
        // TODO: add more data points
        Map.entry(1.0, 1.0),
        Map.entry(4.86, 1.5)
    );

    // meters, RPS
    private static final InterpolatingDoubleTreeMap SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(2.0, 2700.0),
        Map.entry(3.0, 3000.0),
        Map.entry(4.0, 3300.0),
        Map.entry(4.86, 3750.0)
    );

    /*  meters, degrees
    private static final InterpolatingDoubleTreeMap HOOD_ANGLE_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.0, 15.0),
        Map.entry(2.0, 30.0),
        Map.entry(3.0, 45.0)
    );*/

    @Override
    public boolean isFinished(){
        return false;
    }
}
