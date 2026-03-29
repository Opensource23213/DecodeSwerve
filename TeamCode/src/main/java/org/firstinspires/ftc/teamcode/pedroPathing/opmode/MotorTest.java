package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name="Motor Test", group="ABC Opmode")

public class MotorTest extends OpMode {
    public DcMotorEx shoot1;
    public DcMotorEx shoot2;
    public DcMotor spindexer;
    public Servo flippy;
    public Servo turret_servo_1;
    public Servo turret_servo_2;
    public static double test_speed = 0;
    public static double p = 150, i = 1, d = 0.001;
    public Servo flap = null;
    public static double close_pos = 0;
    public static double mid_pos = 0;
    public static double far_pos = 0;
    public static double close_dis = 0;
    public static double mid_dis = 0;
    public static double far_dis = 0;

    public static double f = 7;
    public static double position = 0;
    public DcMotorEx intake = null;

    @Override
    public void init(){
        flap = hardwareMap.get(Servo.class, "flap");
        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        PIDFCoefficients pidf = new PIDFCoefficients(50, 20, 0, 1);
        shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret_servo_1 = hardwareMap.get(Servo.class, "turret_servo_1");
        turret_servo_2 = hardwareMap.get(Servo.class, "turret_servo_2");
        turret_servo_1.setDirection(Servo.Direction.REVERSE);
        turret_servo_2.setDirection(Servo.Direction.REVERSE);
        flippy = hardwareMap.get(Servo.class, "flippy");
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop(){
        shoot1.setVelocity(test_speed);
        shoot2.setVelocity(test_speed);
        spindexer.setPower(gamepad1.right_trigger);
        turret_servo_1.setPosition(.5 + .0062);
        turret_servo_2.setPosition(.5 - .0062);
        flippy.setPosition(.5);
        intake.setPower(gamepad1.left_trigger);
        flap.setPosition(position + .02);
        telemetry.addData("Velocity", shoot1.getVelocity());
        telemetry.update();
    }

}
