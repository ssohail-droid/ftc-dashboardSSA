package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "RPM PIDF Dashboard Extended", group = "Tutorial")
public class RPM_PIDF_Dashboard extends LinearOpMode {

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private DcMotorEx intakeMotor;

    // REV HD Hex encoder (no gearbox)
    private static final double TICKS_PER_REV = 112.0;

    // Tunable PIDF values (from dashboard)
    public static double kP = 20.0;
    public static double kI = 0.0;
    public static double kD = 2.0;
    public static double kF = 12.0;

    // Individual target RPMs
    public static double targetRPM_left = 0;
    public static double targetRPM_right = 0;
    public static double targetRPM_intake = 0;

    // Direction flags (true = REVERSE, false = FORWARD)
    public static boolean leftReverse = false;
    public static boolean rightReverse = true;
    public static boolean intakeReverse = false;

    @Override
    public void runOpMode() {
        // Hardware mapping
        leftMotor = hardwareMap.get(DcMotorEx.class, "shoot");
        rightMotor = hardwareMap.get(DcMotorEx.class, "upPipe");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "midPipe");

        // Reset encoders
        for (DcMotorEx motor : new DcMotorEx[]{leftMotor, rightMotor, intakeMotor}) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            // Set directions from dashboard
            leftMotor.setDirection(leftReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            rightMotor.setDirection(rightReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            intakeMotor.setDirection(intakeReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

            // Update PIDF for all motors
            for (DcMotorEx motor : new DcMotorEx[]{leftMotor, rightMotor, intakeMotor}) {
                motor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            }

            // Convert RPM to ticks/sec
            double leftVelocity = rpmToTicksPerSecond(targetRPM_left);
            double rightVelocity = rpmToTicksPerSecond(targetRPM_right);
            double intakeVelocity = rpmToTicksPerSecond(targetRPM_intake);

            // Set motor velocities
            leftMotor.setVelocity(leftVelocity);
            rightMotor.setVelocity(rightVelocity);
            intakeMotor.setVelocity(intakeVelocity);

            // Telemetry packet for Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target RPM Left", targetRPM_left);
            packet.put("Target RPM Right", targetRPM_right);
            packet.put("Target RPM Intake", targetRPM_intake);

            packet.put("Left Actual Velocity", leftMotor.getVelocity());
            packet.put("Right Actual Velocity", rightMotor.getVelocity());
            packet.put("Intake Actual Velocity", intakeMotor.getVelocity());

            dashboard.sendTelemetryPacket(packet);

            // Driver Station telemetry
            telemetry.addData("Target RPM Left", targetRPM_left);
            telemetry.addData("Target RPM Right", targetRPM_right);
            telemetry.addData("Target RPM Intake", targetRPM_intake);
            telemetry.addData("Left Actual Velocity", leftMotor.getVelocity());
            telemetry.addData("Right Actual Velocity", rightMotor.getVelocity());
            telemetry.addData("Intake Actual Velocity", intakeMotor.getVelocity());
            telemetry.update();
        }
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }
}
