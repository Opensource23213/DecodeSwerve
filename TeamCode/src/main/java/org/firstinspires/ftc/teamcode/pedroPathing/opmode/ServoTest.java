package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Servo Test", group="ABC Opmode")

public class ServoTest extends DecodeLibrary{
    public Servo servo1;
    public Servo servo2;
    public static double test_pos = .5;
    public static double offset = .0062;
    public Servo turret_servo_1;
    public Servo turret_servo_2;
    @Override
    public void init(){
        turret_servo_1 = hardwareMap.get(Servo.class, "turret_servo_1");
        turret_servo_2 = hardwareMap.get(Servo.class, "turret_servo_2");
        turret_servo_1.setDirection(Servo.Direction.REVERSE);
        turret_servo_2.setDirection(Servo.Direction.REVERSE);

    }
    @Override
    public void loop(){
        turret_servo_1.setPosition(test_pos + offset);
        turret_servo_2.setPosition(test_pos - offset );
        telemetry.addData("Servo Position", test_pos);
        telemetry.update();
    }

}
