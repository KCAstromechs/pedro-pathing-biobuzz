package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Basic Motor Test", group = "Linear OpMode")
public class BasicMotorTest extends LinearOpMode {

    // Declare the motor object
    private DcMotor testMotor;

    @Override
    public void runOpMode() {

        // Hardware Mapping: Ensure "test_motor" matches your Robot Controller configuration
        testMotor = hardwareMap.get(DcMotor.class, "flywheel");

        // Set the motor direction (use REVERSE if it turns the wrong way)
        testMotor.setDirection(DcMotor.Direction.FORWARD);

        // Optional: Set motor behavior when power is set to 0 (BRAKE or FLOAT)
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized. Ready to start.");
        telemetry.update();

        // Wait for the driver to press PLAY on the Driver Station
        waitForStart();

        // Run until the driver presses STOP
        while (opModeIsActive()) {

            // If Gamepad 1 'A' button is held, run at full speed; otherwise stop
            if (gamepad1.a) {
                testMotor.setPower(1.0);
            } else {
                testMotor.setPower(0.0);
            }

            // Send feedback to the Driver Station screen
            telemetry.addData("Motor Power", testMotor.getPower());
            telemetry.update();
        }
    }
}