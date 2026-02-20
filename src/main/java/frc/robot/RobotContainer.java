// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.controls.DriverControls;
import frc.robot.controls.OperatorControls;
import frc.robot.controls.PoseControls;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import swervelib.SwerveDrive;

import static edu.wpi.first.units.Units.Inches;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // TODO: configure the swerve modules and get the deploy folder
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final HopperSubsystem hopper = new HopperSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  private final HoodSubsystem hood = new HoodSubsystem();

  private final Superstructure superstructure = new Superstructure(intake, hopper, feeder, turret, shooter, climber, hood);

  private final SendableChooser<Command> autoChooser;

  // track current alliance for change detection
  private Alliance currentAlliance = Alliance.Red;

  // contains subsystems, i/o devices, and commands
  public RobotContainer(){
    // configure trigger bindings
    configureBindings();
    buildNamedAutoCommands();

    // intialize alliance (default to red if not present)
    onAllianceChanged(getAlliance());

    // set up trigger to detect alliance changes
    new Trigger(() -> getAlliance() != currentAlliance)
      .onTrue(Commands.runOnce(() -> onAllianceChanged(getAlliance())).ignoringDisable(true));
    
    // triggers for auto aim/pass poses
    new Trigger(() -> isInAllianceZone())
      .onChange(Commands.runOnce(() -> onZoneChanged()).ignoringDisable(true));
    
    new Trigger(() -> isOnAllianceOutpostSide())
      .onChange(Commands.runOnce(() -> onZoneChanged()).ignoringDisable(true));
      
    if(!Robot.isReal() || true){
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // have autoChooser pull all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    // set default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // add a simple auto option to have the robot drive backward for 1 second then stop
    autoChooser.addOption("Drive Backward", drivebase.driveBackwards().withTimeout(1));
    
    // put autoChooser on SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings(){
    // set up controllers
    DriverControls.configure(Constants.ControllerConstants.kDriverControllerPort, drivebase, superstructure);
    OperatorControls.configure(Constants.ControllerConstants.kOperatorControllerPort, drivebase, superstructure);
    PoseControls.configure(Constants.ControllerConstants.kPoseControllerPort,drivebase);
  }

  private void buildNamedAutoCommands(){
    // add any auto commands to NamedCommands here
    NamedCommands.registerCommand("driveBackwards", drivebase.driveBackwards().withTimeout(1).withName("Auto.driveBackwards"));
    NamedCommands.registerCommand("driveForwards", drivebase.driveForward().withTimeout(1).withName("Auto.driveForwards"));
  
    NamedCommands.registerCommand("aimShooting", superstructure.aimCommand(superstructure.getTargetShooterSpeed(), superstructure.getTargetTurretAngle(), superstructure.getTargetHoodAngle()).withName("Auto.AimCommand"));
    NamedCommands.registerCommand("stopShooting", superstructure.stopAllShootingCommand().withName("Auto.StopShooting"));
    NamedCommands.registerCommand("aimDynamicShooting", superstructure.aimDynamicCommand(() -> shooter.getSpeed(), () -> turret.getRawAngle(), () -> hood.getAngle()));
    
    NamedCommands.registerCommand("feedShooter", superstructure.feedAllCommand().withName("Auto.FeedShooter"));
    NamedCommands.registerCommand("stopFeed", superstructure.stopFeedingAllCommand().withName("Auto.StopFeed"));

    NamedCommands.registerCommand("climbUp", superstructure.moveClimberUp().withName("Auto.ClimbUp"));
    NamedCommands.registerCommand("climbDown", superstructure.moveClimberDown().withName("Auto.ClimbDown"));
    
    NamedCommands.registerCommand("deployIntake", superstructure.setIntakeDeployAndRoll().withName("Auto.DeployIntake"));
    NamedCommands.registerCommand("retractIntake", superstructure.setIntakeStow().withName("Auto.StowIntake"));
    NamedCommands.registerCommand("bounceIntake", superstructure.intakeBounceCommand().withName("Auto.BounceIntake"));
    NamedCommands.registerCommand("eject", superstructure.ejectAllCommand().withName("Auto.Eject"));

    NamedCommands.registerCommand("centerTurret", superstructure.setTurretForward().withName("Auto.CenterTurret"));
    NamedCommands.registerCommand("manualShoot", superstructure.shootCommand().withName("Auto.ManualShoot"));

    NamedCommands.registerCommand("shootOnTheMove", new ShootOnTheMoveCommand(drivebase,superstructure,() -> superstructure.getAimPoint()).ignoringDisable(true).withName("Auto.Eject"));
  }

  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }

  public SwerveDrive getSwerveDrive(){
    return drivebase.getSwerveDrive();
  }

  public Pose2d getRobotPose(){
    return drivebase.getPose();
  }

  public Pose3d getAimDirection(){
    // apply robot heading first, then turret/hood rotation on top
    Pose3d shooterPose = superstructure.getShooterPose();

    var pose = drivebase.getPose3d().plus(new Transform3d(
      shooterPose.getTranslation(), shooterPose.getRotation()
    ));

    return pose;
  }

  public Translation3d getAimPoint(){
    return superstructure.getAimPoint();
  }

  public void setAimPoint(Translation3d aimPoint){
    superstructure.setAimPoint(aimPoint);
  }

  private Alliance getAlliance(){
    return DriverStation.getAlliance().orElse(Alliance.Red);
  }

  private boolean isInAllianceZone(){
    Alliance alliance = getAlliance();
    Distance blueZone = Inches.of(182);
    Distance redZone = Inches.of(469);

    if(alliance == Alliance.Blue && drivebase.getPose().getMeasureX().lt(blueZone)){
      return true;
    }else if(alliance == Alliance.Red && drivebase.getPose().getMeasureX().gt(redZone)){
      return true;
    }

    return false;
  }

  private boolean isOnAllianceOutpostSide(){
    Alliance alliance = getAlliance();
    Distance midline = Inches.of(158.84375);

    if(alliance == Alliance.Blue && drivebase.getPose().getMeasureY().lt(midline)){
      return true;
    }else if(alliance == Alliance.Red && drivebase.getPose().getMeasureY().gt(midline)){
      return true;
    }

    return false;
  }

  private void onZoneChanged(){
    if(isInAllianceZone()){
      superstructure.setAimPoint(Constants.AimPoints.getAllianceHubPosition());
    }else{
      if(isOnAllianceOutpostSide()){
        superstructure.setAimPoint(Constants.AimPoints.getAllianceOutpostPosition());
      }else{
        superstructure.setAimPoint(Constants.AimPoints.getAllianceFarSidePosition());
      }
    }
  }

  private void onAllianceChanged(Alliance alliance){
    currentAlliance = alliance;

    // update aim point based on alliance
    if(alliance == Alliance.Blue){
      superstructure.setAimPoint(Constants.AimPoints.BLUE_HUB.value);
    }else{
      superstructure.setAimPoint(Constants.AimPoints.RED_HUB.value);
    }

    System.out.println("Alliance changed to: " + alliance);
  }
}
