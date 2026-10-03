package MotorPID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * 4-port motor velocity PID tuner for FTC Dashboard.
 * Config names in the robot config: "motor0", "motor1", "motor2", "motor3"
 * Edit everything live in Dashboard -> Configuration -> MotorPIDTuner.
 */
@Config
@TeleOp(name = "Motor PID Tuner", group = "Tuning")
public class MotorPIDTuner extends LinearOpMode {

    // ---- Tunables (live in Dash) ----
    public static double TARGET_RPM = 0;
    public static double TICKS_PER_REV = 28;   //goBILDA 5203 1:1 = 28

    public static double kP = 0.0002;
    public static double kI = 0.0002;
    public static double kD = 0.0;
    public static double kF = 0.000167;                // feedforward, power per RPM (~1/maxRPM). Leave 0 for pure PID

    public static double I_MAX = 0.3;             // clamp on integral term's power contribution

    public static boolean ENABLE_0 = true;
    public static boolean ENABLE_1 = false;
    public static boolean ENABLE_2 = false;
    public static boolean ENABLE_3 = false;

    public static boolean REVERSE_0 = false;
    public static boolean REVERSE_1 = false;
    public static boolean REVERSE_2 = false;
    public static boolean REVERSE_3 = false;

    private static final int N = 4;

    private final DcMotorEx[] motors = new DcMotorEx[N];
    private final double[] integral = new double[N];
    private final double[] lastErr = new double[N];
    private final boolean[] lastReverse = new boolean[N];

    @Override
    public void runOpMode() {
        Telemetry dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        for (int i = 0; i < N; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, "motor" + i);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we do the PID ourselves
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        dashTelemetry.addLine("Ready. Set values in Dash, then press START.");
        dashTelemetry.update();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        double lastTime = timer.seconds();

        while (opModeIsActive()) {
            double now = timer.seconds();
            double dt = now - lastTime;
            lastTime = now;
            if (dt <= 0) dt = 1e-3;

            boolean[] enabled = {ENABLE_0, ENABLE_1, ENABLE_2, ENABLE_3};
            boolean[] reverse = {REVERSE_0, REVERSE_1, REVERSE_2, REVERSE_3};
            double tpr = Math.max(TICKS_PER_REV, 1e-6);

            dashTelemetry.addData("targetRPM", TARGET_RPM);

            for (int i = 0; i < N; i++) {
                if (!enabled[i]) {
                    motors[i].setPower(0);
                    integral[i] = 0;
                    lastErr[i] = 0;
                    continue;
                }

                if (reverse[i] != lastReverse[i]) {
                    motors[i].setDirection(reverse[i]
                            ? DcMotorSimple.Direction.REVERSE
                            : DcMotorSimple.Direction.FORWARD);
                    lastReverse[i] = reverse[i];
                }

                // getVelocity() = ticks/sec, already direction-corrected
                double rpm = motors[i].getVelocity() / tpr * 60.0;
                double err = TARGET_RPM - rpm;

                if (TARGET_RPM == 0) {
                    integral[i] = 0;
                } else {
                    integral[i] += err * dt;
                    if (kI != 0) {
                        double maxInt = I_MAX / Math.abs(kI);
                        integral[i] = Math.max(-maxInt, Math.min(maxInt, integral[i]));
                    }
                }

                double deriv = (err - lastErr[i]) / dt;
                lastErr[i] = err;

                double out = kF * TARGET_RPM + kP * err + kI * integral[i] + kD * deriv;
                out = Math.max(-1.0, Math.min(1.0, out));
                if (TARGET_RPM == 0) out = 0;

                motors[i].setPower(out);

                dashTelemetry.addData("rpm" + i, rpm);
                dashTelemetry.addData("err" + i, err);
                dashTelemetry.addData("pow" + i, out);
            }

            dashTelemetry.update();
        }

        for (DcMotorEx m : motors) m.setPower(0);
    }
}