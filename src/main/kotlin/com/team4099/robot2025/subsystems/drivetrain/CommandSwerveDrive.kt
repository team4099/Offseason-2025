package com.team4099.robot2025.subsystems.drivetrain

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveRequest
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.TunerConstants.TunerSwerveDrivetrain
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import org.team4099.lib.units.base.inSeconds
import java.util.function.Consumer
import java.util.function.Supplier
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Twist2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
class CommandSwerveDrive : TunerSwerveDrivetrain, Subsystem {
  private var simNotifier: Notifier? = null
  private var lastSimTime = 0.0

  /* Keep track if we've ever applied the operator perspective before or not */
  private var hasAppliedOperatorPerspective = false

  /* Swerve requests to apply during SysId characterization */
  val translationCharacterization = SwerveRequest.SysIdSwerveTranslation()
  private val steerCharacterization = SwerveRequest.SysIdSwerveSteerGains()
  private val rotationCharacterization = SwerveRequest.SysIdSwerveRotation()

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private val sysIdRoutineTranslation =
    SysIdRoutine(
      SysIdRoutine.Config(
        null, // Use default ramp rate (1 V/s)
        Units.Volts.of(4.0), // Reduce dynamic step voltage to 4 V to prevent brownout
        null, // Use default timeout (10 s)
        // Log state with SignalLogger class
        Consumer { state: SysIdRoutineLog.State? ->
          SignalLogger.writeString("SysIdTranslation_State", state.toString())
        }
      ),
      Mechanism(
        Consumer { output: Voltage? ->
          setControl(translationCharacterization.withVolts(output))
        },
        null,
        this
      )
    )

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private val m_sysIdRoutineSteer =
    SysIdRoutine(
      SysIdRoutine.Config(
        null, // Use default ramp rate (1 V/s)
        Units.Volts.of(7.0), // Use dynamic voltage of 7 V
        null, // Use default timeout (10 s)
        // Log state with SignalLogger class
        Consumer { state: SysIdRoutineLog.State? ->
          SignalLogger.writeString("SysIdSteer_State", state.toString())
        }
      ),
      Mechanism(
        Consumer { volts: Voltage? -> setControl(steerCharacterization.withVolts(volts)) },
        null,
        this
      )
    )

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private val m_sysIdRoutineRotation =
    SysIdRoutine(
      SysIdRoutine.Config(
        /* This is in radians per second², but SysId only supports "volts per second" */
        Units.Volts.of(Math.PI / 6)
          .per(
            Units
              .Second
          ), /* This is in radians per second, but SysId only supports "volts" */
        Units.Volts.of(Math.PI),
        null, // Use default timeout (10 s)
        // Log state with SignalLogger class
        Consumer { state: SysIdRoutineLog.State? ->
          SignalLogger.writeString("SysIdRotation_State", state.toString())
        }
      ),
      Mechanism(
        Consumer { output: Voltage? ->
          /* output is actually radians per second, but SysId only supports "volts" */
          setControl(rotationCharacterization.withRotationalRate(output!!.`in`(Units.Volts)))
          /* also log the requested output for SysId */
          SignalLogger.writeDouble("Rotational_Rate", output.`in`(Units.Volts))
        },
        null,
        this
      )
    )

  /* The SysId routine to test */
  private val sysIdRoutineToApply = sysIdRoutineTranslation

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  constructor(
    drivetrainConstants: SwerveDrivetrainConstants,
    vararg modules: SwerveModuleConstants<*, *, *>?
  ) : super(drivetrainConstants, *modules) {
    if (Utils.isSimulation()) {
      startSimThread()
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   * 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  constructor(
    drivetrainConstants: SwerveDrivetrainConstants,
    odometryUpdateFrequency: Double,
    vararg modules: SwerveModuleConstants<*, *, *>?
  ) : super(drivetrainConstants, odometryUpdateFrequency, *modules) {
    if (Utils.isSimulation()) {
      startSimThread()
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   * 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   * [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   * theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  constructor(
    drivetrainConstants: SwerveDrivetrainConstants,
    odometryUpdateFrequency: Double,
    odometryStandardDeviation: Matrix<N3?, N1?>,
    visionStandardDeviation: Matrix<N3?, N1?>,
    vararg modules: SwerveModuleConstants<*, *, *>?
  ) : super(
    drivetrainConstants,
    odometryUpdateFrequency,
    odometryStandardDeviation,
    visionStandardDeviation,
    *modules
  ) {
    if (Utils.isSimulation()) {
      startSimThread()
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  fun applyRequest(requestSupplier: Supplier<SwerveRequest?>): Command? {
    return run(Runnable { this.setControl(requestSupplier.get()) })
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by
   * [.m_sysIdRoutineToApply].
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command? {
    return sysIdRoutineToApply.quasistatic(direction)
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by
   * [.m_sysIdRoutineToApply].
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command? {
    return sysIdRoutineToApply.dynamic(direction)
  }

  override fun periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
        .ifPresent(
          Consumer { allianceColor: Alliance? ->
            setOperatorPerspectiveForward(
              if (allianceColor == Alliance.Red) kRedAlliancePerspectiveRotation
              else kBlueAlliancePerspectiveRotation
            )
            hasAppliedOperatorPerspective = true
          }
        )
    }

    CustomLogger.recordOutput("Odometry/pose", state.Pose)
    CustomLogger.recordOutput("Odometry/pose3d", Pose3d(state.Pose))
  }

  private fun startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds()

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
      Notifier(
        Runnable {
          val currentTime = Utils.getCurrentTimeSeconds()
          val deltaTime = currentTime - lastSimTime
          lastSimTime = currentTime

          /* use the measured time delta, get battery voltage from WPILib */
          updateSimState(deltaTime, RobotController.getBatteryVoltage())
        }
      )
    simNotifier!!.startPeriodic(kSimLoopPeriod)
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  override fun addVisionMeasurement(visionRobotPoseMeters: Pose2d?, timestampSeconds: Double) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds))
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * Note that the vision measurement standard deviations passed into this method will continue to
   * apply to future measurements until a subsequent call to [.setVisionMeasurementStdDevs] or this
   * method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
   * [x, y, theta]ᵀ, with units in meters and radians.
   */
  override fun addVisionMeasurement(
    visionRobotPoseMeters: Pose2d?,
    timestampSeconds: Double,
    visionMeasurementStdDevs: Matrix<N3?, N1?>?
  ) {
    super.addVisionMeasurement(
      visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs
    )
  }

  /**
   * function to interrupt control requests to check for minimize skew
   */
  override fun setControl(request: SwerveRequest?) {
    if (!DrivetrainConstants.MINIMIZE_SKEW || request == null) return setControl(request)
    if (!(request is ApplyFieldSpeeds || request is ApplyRobotSpeeds || request is FieldCentric || request is RobotCentric)) return setControl(request)
    when (request) {
      is ApplyFieldSpeeds, is ApplyRobotSpeeds -> {
        val desiredChassisSpeeds = when (request) {
          is ApplyFieldSpeeds -> ChassisSpeeds(request.Speeds)
          is ApplyRobotSpeeds -> ChassisSpeeds(request.Speeds)
          else -> ChassisSpeeds() // unreachable
        }

        val velocityTransform =
          Transform2d(
            Translation2d(
              Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.vx,
              Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.vy
            ),
            Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.omega
          )

        val twistToNextPose: Twist2d = velocityTransform.log()

        setControl(
          when (request) {
            is ApplyFieldSpeeds ->
              request.withSpeeds(
                ChassisSpeeds(
                  (twistToNextPose.dx / Constants.Universal.LOOP_PERIOD_TIME),
                  (twistToNextPose.dy / Constants.Universal.LOOP_PERIOD_TIME),
                  (twistToNextPose.dtheta / Constants.Universal.LOOP_PERIOD_TIME)
                ).chassisSpeedsWPILIB
              )
            is ApplyRobotSpeeds ->
              request.withSpeeds(
                ChassisSpeeds(
                  (twistToNextPose.dx / Constants.Universal.LOOP_PERIOD_TIME),
                  (twistToNextPose.dy / Constants.Universal.LOOP_PERIOD_TIME),
                  (twistToNextPose.dtheta / Constants.Universal.LOOP_PERIOD_TIME)
                ).chassisSpeedsWPILIB
              )
            else -> request // unreachable
          }
        )
      }
      is RobotCentric, is FieldCentric -> {
        val desiredChassisSpeeds = when (request) {
          is RobotCentric -> ChassisSpeeds(request.VelocityX.meters.perSecond, request.VelocityY.meters.perSecond, request.RotationalRate.radians.perSecond)
          is FieldCentric -> ChassisSpeeds.fromFieldRelativeSpeeds(request.VelocityX.meters.perSecond, request.VelocityY.meters.perSecond, request.RotationalRate.radians.perSecond, state.Pose.rotation.radians.radians)
          else -> ChassisSpeeds() // unreachable
        }

        val velocityTransform =
          Transform2d(
            Translation2d(
              Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.vx,
              Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.vy
            ),
            Constants.Universal.LOOP_PERIOD_TIME * desiredChassisSpeeds.omega
          )

        val twistToNextPose: Twist2d = velocityTransform.log()

        val newRequest: SwerveRequest

        when (request) {
          is RobotCentric ->
            newRequest = request
              .withVelocityX((twistToNextPose.dx / Constants.Universal.LOOP_PERIOD_TIME).inMetersPerSecond)
              .withVelocityY((twistToNextPose.dy / Constants.Universal.LOOP_PERIOD_TIME).inMetersPerSecond)
              .withRotationalRate((twistToNextPose.dtheta / Constants.Universal.LOOP_PERIOD_TIME).inRadiansPerSecond)
          is FieldCentric -> {
            // test below math
            val vx = (twistToNextPose.dx / Constants.Universal.LOOP_PERIOD_TIME).inMetersPerSecond
            val vy = (twistToNextPose.dy / Constants.Universal.LOOP_PERIOD_TIME).inMetersPerSecond
            val omega = (twistToNextPose.dtheta / Constants.Universal.LOOP_PERIOD_TIME).inRadiansPerSecond
            newRequest = request
              .withVelocityX( vx * state.Pose.rotation.cos - vy * state.Pose.rotation.sin)
              .withVelocityY(vx * state.Pose.rotation.sin + vy * state.Pose.rotation.cos)
              .withRotationalRate(omega)
          }
          else -> newRequest = request // unreachable
        }

        setControl(newRequest)
      }
    }

  }

  companion object {
    private val kSimLoopPeriod = Constants.Universal.LOOP_PERIOD_TIME.inSeconds

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private val kBlueAlliancePerspectiveRotation: Rotation2d = Rotation2d.kZero

    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private val kRedAlliancePerspectiveRotation: Rotation2d? = Rotation2d.k180deg
  }
}
