/**
 * This is an AI generated sample subsystem that includes potential code for many kinds of mechanisms
 */

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// Optional imports: enable only with the corresponding tagged sections.
// [SERVO] import com.qualcomm.robotcore.hardware.Servo;
// [SENSOR] import com.qualcomm.robotcore.hardware.TouchSensor;
// [TELEMETRY] import org.firstinspires.ftc.robotcore.external.Telemetry;
// [DASHBOARD] import com.acmerobotics.dashboard.config.Config;
// [IVY] import com.pedropathing.ivy.CommandBuilder;
// [IVY] import com.pedropathing.ivy.commands.Commands;

/**
 * Copy, rename, and trim this concrete example for ONE mechanism.
 * The default example regulates motor velocity, exposed in encoder ticks/second and RPM.
 * Configure encoderTicksPerRevolution before using RPM; the hub still receives ticks/second.
 * It requires only the FTC SDK; optional hardware and integrations are commented out.
 *
 * Set requests, call enable(), then call periodic() once per OpMode loop.
 * Call disable() when stopping. No worker thread or scheduler is created here.
 * Zero PIDF gains and a zero manual power limit prevent motion until configured.
 *
 * Enable/remove each tagged feature as a unit: imports, fields, constructor setup,
 * output writes, and methods. When uncommenting code, remove inline tag prefixes;
 * keep explanatory comments commented out.
 * The touch sensor is observation-only, NOT an automatic safety interlock.
 * This is not a universal safety controller; add mechanism-specific limits.
 */
// [DASHBOARD] @Config
public class SampleSubsystem {

    // Example tuning values, NOT calibrated values. Edit here or enable Dashboard.
    // SDK/hub PIDF coefficients use different scaling from a custom power calculation.
    public static double proportionalGain = 0.0;        // Built-in velocity P.
    public static double integralGain = 0.0;            // Built-in velocity I.
    public static double derivativeGain = 0.0;          // Built-in velocity D.
    public static double velocityFeedforwardGain = 0.0; // Built-in velocity F; retune.
    public static double velocityToleranceTicksPerSecond = 0.0; // Set directly or via either unit setter.
    // Positive decoded encoder ticks per MOTOR OUTPUT SHAFT revolution, including its gearbox.
    // Replace 0.0 with your motor's value; no motor-specific conversion is assumed.
    // RPM getters report NaN and RPM setters throw until this is configured.
    public static double encoderTicksPerRevolution = 0.0;
    public static double maximumPower = 0.0;             // MANUAL power cap in [0, 1] only.
    // setVelocity() lets the hub choose output power; maximumPower does NOT cap it.

    private final DcMotorEx primaryMotor;
    // [SECONDARY] private final DcMotorEx secondaryMotor;
    // [SERVO] private final Servo positionServo;
    // [SENSOR] private final TouchSensor stateSensor;

    // Per-instance state: never share live requests between subsystem instances.
    private double targetVelocityTicksPerSecond = 0.0;   // One shared target for both unit APIs.
    private double manualPower = 0.0;
    private boolean enabled = false;
    private boolean manualControl = false;
    private PIDFCoefficients appliedVelocityPIDF;

    public SampleSubsystem(HardwareMap hardwareMap) {
        // Replace every hardware-map name with your Robot Configuration name.
        primaryMotor = hardwareMap.get(DcMotorEx.class, "primaryMotor");
        primaryMotor.setPower(0.0);
        // Example direction only: verify positive power and encoder velocity agree.
        primaryMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        primaryMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // FLOAT coasts at zero power. Choose BRAKE if appropriate; neither holds a load.
        primaryMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // [SECONDARY] Same velocity request; each motor requires its own encoder feedback.
        // [SECONDARY] Match physical directions; this is NOT independent synchronization.
        // secondaryMotor = hardwareMap.get(DcMotorEx.class, "secondaryMotor");
        // secondaryMotor.setPower(0.0);
        // secondaryMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // secondaryMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // secondaryMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // [SERVO] No startup position is assumed safe, so initialization does not move it.
        // positionServo = hardwareMap.get(Servo.class, "positionServo");

        // [SENSOR] Verify the configured device supports TouchSensor.
        // stateSensor = hardwareMap.get(TouchSensor.class, "stateSensor");

        disable();
    }

    /** Select automatic velocity control in ticks/second. Does not enable motion by itself. */
    public void setTargetTicksPerSecond(double ticksPerSecond) {
        requireFinite(ticksPerSecond);
        targetVelocityTicksPerSecond = ticksPerSecond;
        manualControl = false;
        // The new target takes effect on the next periodic() call.
    }

    /** Select automatic velocity control in motor output shaft RPM; preserves direction sign. */
    public void setTargetRpm(double rpm) {
        setTargetTicksPerSecond(rpmToTicksPerSecond(rpm));
    }

    public double getTargetTicksPerSecond() {
        return targetVelocityTicksPerSecond;
    }

    public double getTargetRpm() {
        return ticksPerSecondToRpm(targetVelocityTicksPerSecond);
    }

    /** Shared tolerance setting; either setter updates the same ticks/second value. */
    public static void setVelocityToleranceTicksPerSecond(double ticksPerSecond) {
        requireFinite(ticksPerSecond);
        if (ticksPerSecond < 0.0) {
            throw new IllegalArgumentException("Velocity tolerance must be nonnegative.");
        }
        velocityToleranceTicksPerSecond = ticksPerSecond;
    }

    public static void setVelocityToleranceRpm(double rpm) {
        setVelocityToleranceTicksPerSecond(rpmToTicksPerSecond(rpm));
    }

    public static double getVelocityToleranceTicksPerSecond() {
        return velocityToleranceTicksPerSecond;
    }

    public static double getVelocityToleranceRpm() {
        return ticksPerSecondToRpm(velocityToleranceTicksPerSecond);
    }

    /** Select manual control; apply immediately only if already enabled. */
    public void setManualPower(double power) {
        requireFinite(power);
        manualPower = Math.max(-1.0, Math.min(1.0, power));
        manualControl = true;
        applyMotorPower(enabled ? manualPower : 0.0);
    }

    /** Allow the selected request to run on the next periodic() call. */
    public void enable() {
        enabled = true;
    }

    /** Stop motors immediately and clear requests so re-enabling cannot resume them. */
    public void disable() {
        enabled = false;
        manualControl = false;
        manualPower = 0.0;
        targetVelocityTicksPerSecond = 0.0;
        applyMotorPower(0.0);
        // [SERVO] No new command is issued: it may still move toward/hold its last target.
        // Stopping motor power is not a guarantee of an immediate mechanical stop.
    }

    /** Turning off clears requests; submit a new request before turning back on. */
    public void toggle() {
        if (enabled) {
            disable();
        } else {
            enable();
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isManualControl() {
        return manualControl;
    }

    /** Nonblocking update: call exactly once per normal OpMode loop. */
    public void periodic() {
        if (!enabled) {
            applyMotorPower(0.0);
            return;
        }
        if (manualControl) {
            applyMotorPower(manualPower);
            return;
        }
        if (targetVelocityTicksPerSecond == 0.0) {
            // Explicit stop/coast request, NOT active closed-loop zero-speed braking.
            applyMotorPower(0.0);
            return;
        }

        if (!updateVelocityPIDF()) {
            applyMotorPower(0.0);
            return;
        }
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        primaryMotor.setVelocity(targetVelocityTicksPerSecond);
        // [SECONDARY] Enable/remove with its field, setup, mode, and PIDF configuration.
        // secondaryMotor.setVelocity(targetVelocityTicksPerSecond);
        // The hub runs the velocity PIDF loop; this method only supplies the target.
        // PIDF coefficients retain their native SDK scaling for either target input unit.
        // Position control needs different units, gains, limits, and reference/homing logic.
    }

    /** Current motor velocity in encoder ticks/second, not RPM. */
    public double getVelocityTicksPerSecond() {
        return primaryMotor.getVelocity();
    }

    /** Current motor output shaft RPM; NaN until encoderTicksPerRevolution is configured. */
    public double getVelocityRpm() {
        return ticksPerSecondToRpm(getVelocityTicksPerSecond());
    }

    /** Raw encoder ticks; this class does not reset or establish a physical zero. */
    public int getPositionTicks() {
        return primaryMotor.getCurrentPosition();
    }

    public double getCurrentAmps() {
        return primaryMotor.getCurrent(CurrentUnit.AMPS);
    }

    /** SDK-reported power; not a measurement of torque, speed, or instantaneous PIDF output. */
    public double getAppliedPower() {
        return primaryMotor.getPower();
    }

    public double getErrorTicksPerSecond() {
        return targetVelocityTicksPerSecond - getVelocityTicksPerSecond();
    }

    public double getErrorRpm() {
        return ticksPerSecondToRpm(getErrorTicksPerSecond());
    }

    /** Instantaneous primary-motor check; false when disabled or in manual control. */
    public boolean atTarget() {
        double error = getErrorTicksPerSecond();
        return enabled && !manualControl
                && isFinite(error) && isFinite(velocityToleranceTicksPerSecond)
                && velocityToleranceTicksPerSecond >= 0.0
                && Math.abs(error) <= velocityToleranceTicksPerSecond;
    }

    private void applyMotorPower(double requestedPower) {
        // Manual commands and stopping use open-loop mode, not velocity PIDF.
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double limit = isFinite(maximumPower)
                ? Math.max(0.0, Math.min(1.0, maximumPower)) : 0.0;
        double limitedPower = enabled && isFinite(requestedPower)
                ? Math.max(-limit, Math.min(limit, requestedPower)) : 0.0;
        primaryMotor.setPower(limitedPower);
        // [SECONDARY] Enable/remove this write together with its field and initialization.
        // secondaryMotor.setPower(limitedPower);
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        if (primaryMotor.getMode() != mode) {
            primaryMotor.setPower(0.0);
            primaryMotor.setMode(mode);
        }
        // [SECONDARY] Enable/remove together with all other secondary motor sections.
        // if (secondaryMotor.getMode() != mode) {
        //     secondaryMotor.setPower(0.0);
        //     secondaryMotor.setMode(mode);
        // }
    }

    private boolean updateVelocityPIDF() {
        // Preserve zero-output defaults; reject invalid or untuned coefficients.
        if (!isFinite(proportionalGain) || proportionalGain < 0.0
                || !isFinite(integralGain) || integralGain < 0.0
                || !isFinite(derivativeGain) || derivativeGain < 0.0
                || !isFinite(velocityFeedforwardGain) || velocityFeedforwardGain < 0.0
                || (proportionalGain == 0.0 && integralGain == 0.0
                    && derivativeGain == 0.0 && velocityFeedforwardGain == 0.0)) {
            return false;
        }
        // Apply live tuning changes only when needed, avoiding repeated hub writes.
        if (appliedVelocityPIDF == null
                || appliedVelocityPIDF.p != proportionalGain
                || appliedVelocityPIDF.i != integralGain
                || appliedVelocityPIDF.d != derivativeGain
                || appliedVelocityPIDF.f != velocityFeedforwardGain) {
            PIDFCoefficients coefficients = new PIDFCoefficients(
                    proportionalGain, integralGain, derivativeGain, velocityFeedforwardGain);
            primaryMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
            // [SECONDARY] Requires compatible motors; tune separately if needed.
            // secondaryMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
            appliedVelocityPIDF = coefficients;
        }
        return true;
    }

    private static double rpmToTicksPerSecond(double rpm) {
        requireFinite(rpm);
        double ticksPerRevolution = encoderTicksPerRevolution;
        if (!isFinite(ticksPerRevolution) || ticksPerRevolution <= 0.0) {
            throw new IllegalStateException("Set encoderTicksPerRevolution to a positive finite value.");
        }
        double ticksPerSecond = rpm / 60.0 * ticksPerRevolution;
        requireFinite(ticksPerSecond);
        return ticksPerSecond;
    }

    private static double ticksPerSecondToRpm(double ticksPerSecond) {
        double ticksPerRevolution = encoderTicksPerRevolution;
        if (!isFinite(ticksPerRevolution) || ticksPerRevolution <= 0.0) {
            return Double.NaN; // Allows ticks/second telemetry to work before RPM is configured.
        }
        return ticksPerSecond / ticksPerRevolution * 60.0;
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static void requireFinite(double value) {
        if (!isFinite(value)) {
            throw new IllegalArgumentException("Control requests must be finite.");
        }
    }

    // [SERVO] Enable/remove with Servo import, field, and constructor mapping.
    // Commands are allowed only while enabled. Choose mechanism-specific travel limits.
    // public void setServoPosition(double position) {
    //     requireFinite(position);
    //     if (enabled) {
    //         positionServo.setPosition(Math.max(0.0, Math.min(1.0, position)));
    //     }
    // }
    // public double getServoCommandedPosition() {
    //     return positionServo.getPosition(); // Commanded position, NOT physical feedback.
    // }

    // [SENSOR] Enable/remove with TouchSensor import, field, and constructor mapping.
    // public boolean isStateActive() {
    //     return stateSensor.isPressed(); // Observation only; add any required interlock.
    // }

    // [TELEMETRY] Enable/remove with Telemetry import. Call separately from the OpMode.
    // The caller owns telemetry.update() and the desired reporting frequency.
    // public void addTelemetry(Telemetry telemetry) {
    //     telemetry.addData("Sample enabled", enabled);
    //     telemetry.addData("Sample manual control", manualControl);
    //     telemetry.addData("Sample target ticks/s", getTargetTicksPerSecond());
    //     telemetry.addData("Sample target RPM", getTargetRpm());
    //     telemetry.addData("Sample velocity ticks/s", getVelocityTicksPerSecond());
    //     telemetry.addData("Sample velocity RPM", getVelocityRpm());
    //     telemetry.addData("Sample error ticks/s", getErrorTicksPerSecond());
    //     telemetry.addData("Sample error RPM", getErrorRpm());
    //     telemetry.addData("Sample tolerance ticks/s", getVelocityToleranceTicksPerSecond());
    //     telemetry.addData("Sample tolerance RPM", getVelocityToleranceRpm());
    //     telemetry.addData("Sample position ticks", getPositionTicks());
    //     telemetry.addData("Sample current A", getCurrentAmps());
    //     telemetry.addData("Sample power", getAppliedPower());
    // }

    // [IVY] Optional instant command factories: enable with both Ivy imports.
    // These mirror the CommandBuilder API in the first source snapshot.
    // They change requests only; reaching a target requires later periodic() calls.
    // The other snapshot returns Command and uses infinite(...) for periodic work.
    // Verify YOUR Ivy version; do not mix return-type conventions or run two updates.
    // public CommandBuilder targetCommandTicksPerSecond(double ticksPerSecond) {
    //     requireFinite(ticksPerSecond);
    //     return Commands.instant(() -> setTargetTicksPerSecond(ticksPerSecond));
    // }
    // public CommandBuilder targetCommandRpm(double rpm) {
    //     rpmToTicksPerSecond(rpm); // Validate now; convert again using configuration at execution.
    //     return Commands.instant(() -> setTargetRpm(rpm));
    // }
    // public CommandBuilder enableCommand() {
    //     return Commands.instant(this::enable);
    // }
    // public CommandBuilder disableCommand() {
    //     return Commands.instant(this::disable);
    // }
}
