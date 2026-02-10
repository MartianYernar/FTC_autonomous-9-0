package org.firstinspires.ftc.teamcode;

import android.graphics.HardwareRenderer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpNew")
public class TeleOpNew extends LinearOpMode {
    private DcMotor RightBack; //frontleft 0, frontright 3, backleft 1, backright 2
    private DcMotor RightFront;
    private DcMotor RightIntake;
    private DcMotor LeftIntake;
    private DcMotor RightShooter;
    private DcMotor LeftShooter;
    private DcMotor LeftFront;
    private DcMotor LeftBack;
    double integralSum = 0;
    double Kp = 0.0005;   // уменьшено
    double Ki = 0.0;
    double Kd = 0.0001;

    double lastError = 0;
    private Servo servolid;
    double lastPosition = 0;
    double lastTime = 0;
    boolean shootrot = true;
    double power = 0;

    public void initHardware() {
        servolid.setPosition(0.7);
    }
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() {
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightIntake = hardwareMap.get(DcMotor.class, "RightIntake");
        LeftIntake = hardwareMap.get(DcMotor.class, "LeftIntake");
        RightShooter = hardwareMap.get(DcMotor.class, "RightShooter");
        LeftShooter = hardwareMap.get(DcMotor.class, "LeftShooter");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        servolid = hardwareMap.get(Servo.class, "servolid");
        RightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //added
        lastPosition = RightShooter.getCurrentPosition();
        lastTime = getRuntime();
        // added
        timer.reset();
        initHardware();
        waitForStart();
        RightBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.REVERSE);

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                LeftIntake.setPower(-1);
                RightIntake.setPower(1);
            } else if (gamepad1.right_bumper) {
                LeftIntake.setPower(1);
                RightIntake.setPower(-1);
            } else {
                LeftIntake.setPower(0);
                RightIntake.setPower(0);
            }
            if (gamepad1.y) {
                servolid.setPosition(0.8);
            }
            else if (gamepad1.x) {
                servolid.setPosition(0.7);
            }

            shootrot= true;

            if (shootrot){
                // shooter power
                power = shoot();
                RightShooter.setPower(power);
                LeftShooter.setPower(-power);
                telemetry.addData("Power", power);
                telemetry.update();
            }
            LeftFront.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            LeftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x);
            RightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x);
            RightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x);
        }
    }
    public double shoot() {
        double currentPosition = RightShooter.getCurrentPosition();
        double currentTime = getRuntime();

        double deltaPosition = currentPosition - lastPosition;   // ticks
        double dt = currentTime - lastTime;               // seconds

        double velocity = deltaPosition / dt;             // ticks/sec

        lastPosition = currentPosition;
        lastTime = currentTime;

        // целевое значение (пример)
        double target = 1100;
        double voltage = hardwareMap.get(VoltageSensor.class, "Control Hub").getVoltage();
        telemetry.addData("Voltage", voltage);
        if (voltage < 12.4){
            target = target - 40; // sure
        }
        else if (voltage < 13){
            target = target - 50;
        }
        else if (voltage < 14) {
            target = target - 60;
        }
        else if (voltage > 14) {
            target = target - 70;
        }
        double power = PIDcontrol(target, velocity, dt);

        // ограничение мощности чтобы мотор не улетал на 100%
        power = Math.max(-1, Math.min(1, power));
        return power;
    }
    public double PIDcontrol(double reference, double state, double dt) {
        double error = reference - state;

//        double dt = timer.seconds();
//        timer.reset();

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;

        lastError = error;
        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) result = Math.min(result, voltage);
        }
        return result;
    }
}
