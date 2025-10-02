package com.team4099.robot2025.subsystems.drivetrain

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.TunerConstants.TunerSwerveDrivetrain
import com.team4099.robot2025.util.CustomLogger
import com.team4099.utils.MapleSimSwerveDrivetrain
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Pounds
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inSeconds
import java.util.function.Consumer
import java.util.function.Supplier

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
class CommandSwerveDrive : TunerSwerveDrivetrain, Subsystem {
  private var mapleSimSwerveDrivetrain: MapleSimSwerveDrivetrain? = null
  private var simNotifier: Notifier? = null
  private var resetPoseWaitingTime: Long = 0L
  private var resettingPose = false

  /* Keep track if we've ever applied the operator perspective before or not */
  private var hasAppliedOperatorPerspective = false

  /* Swerve requests to apply during SysId characterization */
  private val translationCharacterization = SwerveRequest.SysIdSwerveTranslation()
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
  ) : super(drivetrainConstants, *MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)) {
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

      // Note(Aryan): Replaced *modules parameter in super call with spread outputted by
      // MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)
      // Change back if having problems with constants or smth similar
  ) : super(drivetrainConstants, odometryUpdateFrequency, *MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)) {
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
    *MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)
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

  /**
   * Adds extra logic to the CTRE resetPose method with to sync up simulation and real-world poses in maplesim
   *
   * @param pose Current pose
   */
  override fun resetPose(pose: Pose2d?) {
    if (this.mapleSimSwerveDrivetrain != null) {
      mapleSimSwerveDrivetrain!!.mapleSimDrive.setSimulationWorldPose(pose)
    }

    // TODO: Find a better way to do this cuz this is kinda jank 😓 (preferably using delay somehow)
    if (!resettingPose) {
      resettingPose = true
      resetPoseWaitingTime = System.currentTimeMillis()
      return
    }

    // Wait 50ms for simulation to update and then sync
    if (System.currentTimeMillis() - resetPoseWaitingTime < 50) {
      return
    }

    resettingPose = false
    super.resetPose(pose)

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

    CustomLogger.recordOutput("Drivetrain/chassisSpeeds", state.Speeds)
    CustomLogger.recordOutput("Drivetrain/timestamp", state.Timestamp)
  }

  private fun startSimThread() {
    mapleSimSwerveDrivetrain = MapleSimSwerveDrivetrain(
      Seconds.of(kSimLoopPeriod),
      Pounds.of(135.0),
      Inches.of(DrivetrainConstants.DRIVETRAIN_LENGTH.inInches),
      Inches.of(DrivetrainConstants.DRIVETRAIN_WIDTH.inInches),
      DCMotor.getKrakenX60(1), // drive motor type
      DCMotor.getKrakenX60(1), // steer motor type
      DrivetrainConstants.NITRILE_WHEEL_COF,
      moduleLocations,
      pigeon2,
      modules,
      TunerConstants.FrontLeft,
      TunerConstants.FrontRight,
      TunerConstants.BackLeft,
      TunerConstants.BackRight
    )

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier = Notifier(mapleSimSwerveDrivetrain!!::update)
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

  companion object {
    private val kSimLoopPeriod = Constants.Universal.LOOP_PERIOD_TIME.inSeconds

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private val kBlueAlliancePerspectiveRotation: Rotation2d = Rotation2d.kZero

    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private val kRedAlliancePerspectiveRotation: Rotation2d? = Rotation2d.k180deg
  }
}
