package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Slider Control", group = "Linear OpMode")
public class SliderControl extends LinearOpMode {

    private DcMotorEx sliderMotor;
    private DigitalChannel limitSwitch;

    // Soft limits in encoder ticks
// Adjust MAX_EXTENSION_TICKS based on your physical slide setup
    private final int MIN_EXTENSION_TICKS = 0;
    private final int MAX_EXTENSION_TICKS = 200;

    @Override
    public void runOpMode() {
        sliderMotor = hardwareMap.get(DcMotorEx.class, "sliderMotor");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

// Set digital channel as an input for the limit switch
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

// Configure motor behavior
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

// Reverse direction if moving power > 0 retracts instead of extends
        sliderMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

// Step 1: Zero out the slider on start using the limit switch
        homeSlider();

        while (opModeIsActive()) {
            double stickInput = -gamepad1.left_stick_y; // Push stick up to extend, down to retract
            int currentPosition = sliderMotor.getCurrentPosition();

// Check limit switch state (REV limit switches read false when pressed)
            boolean isAtBottom = !limitSwitch.getState();

            if (isAtBottom) {
// Keep encoder position zeroed while physical switch is triggered
                sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

// Apply software limits to prevent driver over-extension or forcing past bottom
            if (stickInput > 0 && currentPosition >= MAX_EXTENSION_TICKS) {
// Prevent extending further than max ticks
                sliderMotor.setPower(0);
            } else if (stickInput < 0 && isAtBottom) {
// Prevent retracting further if limit switch is already pressed
                sliderMotor.setPower(0);
            } else {
                sliderMotor.setPower(stickInput);
            }

// Telemetry feedback (if you don't push to git im gonna kill you - from feighny)
            telemetry.addData("Slider Encoder Position", currentPosition);
            telemetry.addData("Limit Switch Pressed", isAtBottom);
            telemetry.addData("Motor Power", sliderMotor.getPower());
            telemetry.update();
        }
    }

    /**
     * Retracts the slider until the limit switch is triggered,
     * then resets the encoder count to zero.
     */
    private void homeSlider() {
        telemetry.addData("Status", "Homing Slider...");
        telemetry.update();

// Move slider downwards slowly to find the limit switch
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderMotor.setPower(-0.3);

// REV Magnetic/Touch Switches are ACTIVE LOW (getState() returns false when pressed)
        while (opModeIsActive() && limitSwitch.getState()) {
            idle();
        }

// Stop motor and reset encoder count to zero
        sliderMotor.setPower(0);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Homing Complete.");
        telemetry.update();
    }
}