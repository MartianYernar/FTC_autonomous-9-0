package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "redclose_Pedro")
public class redclose extends OpMode {
    private DcMotor RightIntake;
    private DcMotor LeftIntake;
    private DcMotor RightShooter;
    private DcMotor LeftShooter;
    double integralSum = 0;
    double Kp = 0.0005;   // уменьшено
    double Ki = 0.0;
    double Kd = 0.0001;
    double lastError = 0;
    private Servo servolid;
    double lastPosition = 0;
    double lastTime = 0;
    double power = 0;
    ElapsedTime timer = new ElapsedTime();
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE ARTIFACT
        DRIVE_START_SHOOT_POS, //startpos
        SHOOT_START_DRIVE_POS, // move back
        DRIVEBACKLOADBALLS1,
        DRIVEFORTHLOADBALLS1,
        DRIVEBACK2LOAD1,
        DRIVEFORTHSHOOTLOAD1,
        DRIVEBACK1LOAD2,
        DRIVEFORTHLOAD2,
        DRIVEBACK2LOAD2,
        DRIVEFORTHSHOOTLOAD2,
        LEAVEEND
    }
    PathState pathState;
    private final Pose startPose = new Pose(110.45, 135.125, Math.toRadians(270));
    private final Pose shootPose = new Pose(105.07500000000003, 104.39999999999999, Math.toRadians(50));
    private final Pose drivebackload1 = new Pose(99.55000000000001, 83.57499999999999, Math.toRadians(0));
    private final Pose driveforthload1 = new Pose(123.82499999999999, 83.57499999999999, Math.toRadians(0));
    private final Pose driveback2load1 = new Pose(99.55000000000001, 83.57499999999999, Math.toRadians(0));
    private final Pose driveforthshootload1 = new Pose(105.07500000000003, 104.39999999999999, Math.toRadians(50));
    private final Pose driveback1load2 = new Pose(98.825, 59.35, Math.toRadians(0));
    private final Pose driveforthload2 = new Pose(125.2, 59.35, Math.toRadians(0));
    private final Pose driveback2load2 = new Pose(98.825, 59.35, Math.toRadians(0));
    private final Pose driveforthshootload2 = new Pose(105.07500000000003, 104.39999999999999, Math.toRadians(50));
    private final Pose leave = new Pose(121.32500000000002, 92.65, Math.toRadians(0));
    private PathChain driveStartPosShootPos, drivebackloadball1, driveforthloadball1, driveback2loadball1, driveforthshootloadball1, driveback1loadball2, driveforthloadball2, driveback2loadball2, driveforthshootloadball2, leaveball;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        // LOAD 1
        drivebackloadball1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, drivebackload1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), drivebackload1.getHeading())
                .build();
        driveforthloadball1 = follower.pathBuilder()
                .addPath(new BezierLine(drivebackload1, driveforthload1))
                .setLinearHeadingInterpolation(drivebackload1.getHeading(), driveforthload1.getHeading())
                .build();
        driveback2loadball1 = follower.pathBuilder()
                .addPath(new BezierLine(driveforthload1, driveback2load1))
                .setLinearHeadingInterpolation(driveforthload1.getHeading(), driveback2load1.getHeading())
                .build();
        driveforthshootloadball1 = follower.pathBuilder()
                .addPath(new BezierLine(driveback2load1, driveforthshootload1))
                .setLinearHeadingInterpolation(driveback2load1.getHeading(), driveforthshootload1.getHeading())
                .build();
        // LOAD 2
        driveback1loadball2 = follower.pathBuilder()
                .addPath(new BezierLine(driveforthshootload1, driveback1load2))
                .setLinearHeadingInterpolation(driveforthshootload1.getHeading(), driveback1load2.getHeading())
                .build();
        driveforthloadball2 = follower.pathBuilder()
                .addPath(new BezierLine(driveback1load2, driveforthload2))
                .setLinearHeadingInterpolation(driveback1load2.getHeading(), driveforthload2.getHeading())
                .build();
        driveback2loadball2 = follower.pathBuilder()
                .addPath(new BezierLine(driveforthload2, driveback2load2))
                .setLinearHeadingInterpolation(driveforthload2.getHeading(), driveback2load2.getHeading())
                .build();
        driveforthshootloadball2 = follower.pathBuilder()
                .addPath(new BezierLine(driveback2load2, driveforthshootload2))
                .setLinearHeadingInterpolation(driveback2load2.getHeading(), driveforthshootload2.getHeading())
                .build();
        leaveball = follower.pathBuilder()
                .addPath(new BezierLine(driveforthshootload2, leave))
                .setLinearHeadingInterpolation(driveforthshootload2.getHeading(), leave.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_SHOOT_POS:
                // shoot
                shoot(1100);
                RightShooter.setPower(power);
                LeftShooter.setPower(-power);
                shoot(1100);
                // move
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_START_DRIVE_POS); // reset the timer & make sound
                break;
            case SHOOT_START_DRIVE_POS:
                // check is follower done its path?
                // check that 5 seconds has elapsed
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    // shooter PID activate or already activated
                    shoot(1100);
                    sleep(3500);
                    servolid.setPosition(0.8);//opened lid
                    intakequickshots(); //intake

                    follower.followPath(drivebackloadball1, true);
                    setPathState(PathState.DRIVEBACKLOADBALLS1);
                    telemetry.addLine("Done path 1");
                }
                break;
            case DRIVEBACKLOADBALLS1:
                if (!follower.isBusy()) {
                    servolid.setPosition(0.7); //closed lid
                    follower.followPath(driveforthloadball1, true);
                    setPathState(PathState.DRIVEFORTHLOADBALLS1);
                    telemetry.addLine("Done path 2");
                }
            case DRIVEFORTHLOADBALLS1:
                if (!follower.isBusy()) {
                    follower.followPath(driveback2loadball1, true);
                    setPathState(PathState.DRIVEBACK2LOAD1);
                    telemetry.addLine("Done path 3");
                }
            case DRIVEBACK2LOAD1:
                if (!follower.isBusy()) {
                    follower.followPath(driveforthshootloadball1, true);
                    setPathState(PathState.DRIVEFORTHSHOOTLOAD1);
                    telemetry.addLine("Done path 4");
                }
            case DRIVEFORTHSHOOTLOAD1:
                if (!follower.isBusy()) {
                    shoot(1100);
                    sleep(3500);
                    servolid.setPosition(0.8);//opened lid
                    intakequickshots(); //intake
                    follower.followPath(driveback1loadball2, true);
                    setPathState(PathState.DRIVEBACK1LOAD2);
                    telemetry.addLine("Done path 5");
                }
                // LOAD 2
            case DRIVEBACK1LOAD2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(driveforthloadball2, true);
                    setPathState(PathState.DRIVEFORTHLOAD2);
                    telemetry.addLine("Done path 6");
                }
            case DRIVEFORTHLOAD2:
                if (!follower.isBusy()) {
                    servolid.setPosition(0.7);

                    LeftIntake.setPower(-1);
                    RightIntake.setPower(1);

                    follower.followPath(driveback2loadball2, true);
                    setPathState(PathState.DRIVEBACK2LOAD2);
                    telemetry.addLine("Done path 7");
                }
            case DRIVEBACK2LOAD2:
                if (!follower.isBusy()) {
                    LeftIntake.setPower(0);
                    RightIntake.setPower(0);
                    follower.followPath(driveforthshootloadball2, true);
                    setPathState(PathState.DRIVEFORTHSHOOTLOAD2);
                    telemetry.addLine("Done path 8");
                }
            case DRIVEFORTHSHOOTLOAD2:
                if (!follower.isBusy()) {
                    shoot(1100);
                    sleep(3500);
                    servolid.setPosition(0.8);//opened lid
                    intakequickshots(); //intake
                    follower.followPath(leaveball, true);
                    setPathState(PathState.LEAVEEND);
                    telemetry.addLine("Done path 9");
                }
            case LEAVEEND:
                if (!follower.isBusy()) {
//                    follower.followPath(drive, true);
//                    setPathState(PathState.DRIVEBACKLOADBALLS1);
                    telemetry.addLine("LEAVE END");
                }
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_START_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
//        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisms
        buildPaths();
        follower.setPose(startPose);
        // motors
        RightIntake = hardwareMap.get(DcMotor.class, "RightIntake");
        LeftIntake = hardwareMap.get(DcMotor.class, "LeftIntake");
        RightShooter = hardwareMap.get(DcMotor.class, "RightShooter");
        LeftShooter = hardwareMap.get(DcMotor.class, "LeftShooter");
        servolid = hardwareMap.get(Servo.class, "servolid");
        RightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servolid.setPosition(0.7);
        lastPosition = RightShooter.getCurrentPosition();
        lastTime = getRuntime();
        timer.reset();
        // shoot calibration
        RightShooter.setPower(0.2);
        LeftShooter.setPower(-0.2);
    }
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }
    @Override
    public void loop() {
        // move
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());


    }
    public void shoot(double target){
        double currentPosition = RightShooter.getCurrentPosition();
        double currentTime = getRuntime();

        double deltaPosition = currentPosition - lastPosition;   // ticks
        double dt = currentTime - lastTime;               // seconds

        double velocity = deltaPosition / dt;             // ticks/sec

        lastPosition = currentPosition;
        lastTime = currentTime;

        // целевое значение (пример)
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
        RightShooter.setPower(power);
        LeftShooter.setPower(-power);
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
    public void intakeactivmilsec(int ms){
        LeftIntake.setPower(-1);
        RightIntake.setPower(1);
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        LeftIntake.setPower(0);
        RightIntake.setPower(0);
    }
    public void sleep(int ms){
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void intakequickshots() {
        intakeactivmilsec(400);
        sleep(400);

        intakeactivmilsec(400);
        sleep(400);

        intakeactivmilsec(400);
        sleep(400);
    }
}
