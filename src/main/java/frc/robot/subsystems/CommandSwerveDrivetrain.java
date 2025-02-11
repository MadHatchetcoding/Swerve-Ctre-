package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;
    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    double targetX = 0;
    double targetY = 0;

    private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, 
            Volts.of(4),
            null,
            (state)-> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> setControl(TranslationCharacterization.withVolts(volts)), null, this));

    private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                    (volts)-> setControl(RotationCharacterization.withVolts(volts)),
                    null,
                    this));

          private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(7),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                    (volts)-> setControl(RotationCharacterization.withVolts(volts)),
                    null,
                    this));
     private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public Command getAutoPath(String pathname){
        return new PathPlannerAuto(pathname);
    }

    private void configurePathPlanner(){
        double driveBaseRadius = 0;
        for(var muoduleLocation: m_moduleLocations){
            driveBaseRadius = Math.max(driveBaseRadius, muoduleLocation.getNorm());

        }
        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose,
            this::seedFieldRelative,
            this::getCurrentRobotChassisSpeeds, 
            (speeds) ->this.setControl(AutoRequest.withSpeeds(speeds)),
            new HolonomicPathFollowerConfig(new PIDConstants(2, 0, 0.1), 
            new PIDConstants(4, 0, 0.1), TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()),Robot::isRed,this);
        
    }


    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command sysIdQuaistatic(SysIdRoutine.Direction direction){
        return RoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return RoutineToApply.quasistatic(direction);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds(){
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Rotation2d directionToGoal()
    {
        Pose3d pos = LimelightHelpers.getBotPose3d("limelight");
        double botX = pos.getX();  
        double botY = pos.getY();  
        Pose2d botPose = new Pose2d(botX, botY, Rotation2d.fromDegrees(0));
        DriverStation.getAlliance().ifPresent((allianceColor) -> {
              if(allianceColor != Alliance.Red)
              {
                
                targetX = -8.135;
                targetY = 1.245;
                
              }
              else
              {
                targetX = 8.135;
                targetY = 1.245;
                
              }
        });
           Pose2d targetPose = new Pose2d(targetX, targetY, Rotation2d.fromDegrees(0));
        Translation2d targetAngle = targetPose.getTranslation();
        Rotation2d rotationNeeded = targetAngle.minus(botPose.getTranslation()).getAngle();
        rotationNeeded.minus(Rotation2d.fromDegrees(180));
        //rotationNeeded.div(-1);
        SmartDashboard.putNumber("Rotation To Goal", rotationNeeded.getDegrees());
        SmartDashboard.putNumber("Gyro", m_pigeon2.getYaw().getValue());
        return rotationNeeded;
    }


    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
}
