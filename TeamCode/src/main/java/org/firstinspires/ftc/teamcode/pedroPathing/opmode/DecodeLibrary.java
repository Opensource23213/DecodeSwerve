package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.SwerveConst;

import java.util.ArrayList;
import java.util.List;

@Config
//@Disabled
public class DecodeLibrary extends OpMode {
    public static double color = 0;
    public ElapsedTime slow = new ElapsedTime();
    public ElapsedTime fast = new ElapsedTime();
    public double robot_x = 0;
    public double robot_y = 0;
    public double robot_heading = 0;
    public static boolean index_reverse = true;
    public static double speed_far = 1260;
    public boolean manual_turret = false;
    public static double launch_angle = 30;
    public static double spin_speed = 1;
    public boolean first_count = false;
    public DcMotorEx spindexer;
    public static Pose auto_pose = new Pose();
    public static double tele_offset = 0;

    public List<Double> balls = new ArrayList<>();
    public static double pattern = 1;
    public static double angle_offset = -90;
    public static double moving_offset = 1.4;
    public static double robot_velocity_stop_shoot = 10;
    public double old_dis = 0;
    public DcMotorEx intake = null;
    public shooter shooter = new shooter();
    public turret turret = new turret();
    public sensors sensors = new sensors();
    public CameraCode cameraCode = new CameraCode();
    public static double adjust = 0.01;
    public static double flippy_up = 0.49;
    public static double flippy_hold = .45;
    public double  flippy_down = .29;
    public double flippy_pos = flippy_down;
    public Servo flippy = null;
    public static double sppeed = 0;
    public static double fllap = .04;
    public static double shoot_offset = -4;
    public ElapsedTime flip_up = new ElapsedTime();
    public static double flap_minimum = .06;
    public static double shoot_power_offset = 1670;
    public static double shoot_multiplier = 410;
    /*
    public static double shoot_power_offset = 1360;
    public static double shoot_multiplier = 240;
     */
    public boolean teleop = true;
    public double index_steps = 0;
    public static double flap_switch = -40;
    public static double flap_height = 0;
    public static double shot_time = 200;
    public List<LynxModule> hubs = null;
    @Override
    public void init(){
        initialize();
    }

    public void initialize() {
        index_reverse = teleop;
        x_mod = 3;
        y_mod = -7;
        follower = SwerveConst.createFollower(hardwareMap, gamepad1, color);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flippy = hardwareMap.get(Servo.class, "flippy");
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(!teleop) {
            cameraCode = new CameraCode();
            cameraCode.init();
        }
        shooter.initialize();
        turret.initialize();
        sensors.initialize();
        follower.update();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
    }
    public Button1 button1 = new Button1();
    public class Button1{
        List<String> button = new ArrayList<>();
        List<String> nowbutton = new ArrayList<>();
        List<String> lastbutton = new ArrayList<>();
        String type = "";
        public void button(){
            ButtonControl();
        }
        public void ButtonControl(){

            if(gamepad1.leftBumperWasReleased()) {
                balls.clear();
                balls.add(1.0);
                balls.add(2.0);
                balls.add(3.0);
            }
            if(gamepad2.touchpadWasPressed()){
                manual_turret = true;
            }

            if(gamepad1.a){
                intake.setPower(0);
            }
            if(gamepad1.dpadRightWasReleased() || gamepad2.dpadRightWasReleased()){
                if(color == 0) {
                    y_mod -= 1;
                }else{
                    y_mod += 1;
                }
            }else if(gamepad1.dpadLeftWasReleased() || gamepad2.dpadLeftWasReleased()){
                if(color == 1) {
                    y_mod -= 1;
                }else{
                    y_mod += 1;
                }
            }
            if(gamepad1.dpadUpWasReleased() || gamepad2.dpadUpWasReleased()){
                x_mod += 1;
            }else if(gamepad1.dpadDownWasReleased() || gamepad2.dpadDownWasReleased()){
                x_mod -= 1;
            }


        }

    }


    public boolean robot_going_forward = true;
    public Follower follower;

    public boolean freeze = false;
    public static double turn_divider = 2;
    public boolean start = false;
    double times = 0;
    boolean back = false;
    public static double amps = 2000;
    public static double loop_time = 100;
    public void teleop_loop(){
        if(start){
            for(LynxModule hub : hubs){
                hub.clearBulkCache();
            }
            follower.update();
            double[] doubles = new double[0];
            if(gamepad1.left_trigger < .4  || sensors.last_time()) {
                freeze = false;
            }else{
                follower.drivetrain.setYVelocity(0);
                freeze = true;
            }



            sensors.sense();
            button1.button();
            fast.reset();
            if(gamepad1.ps){
                follower.setPose(new Pose());
            }
            turret.turret_move();
            telemetry.addData("angle", Math.toDegrees(location.getHeading()));
            shooter.speed = shoot_multiplier * ((dead_distance * .0254) - 1.6) + shoot_power_offset;
            shooter.shooting();
            flippy.setPosition(flippy_pos);
            slow.reset();

        }else if(!gamepad1.atRest()){
            start = true;
        }
    }
    public class CameraCode{
        public Limelight3A limelight;
        public double x = 0;
        public double y = 0;
        public double r = 0;
        public double z = 0;
        public double ticks_per_degree = 5.228;
        public double tag_to_target_distance = .47;
        public double distance_from_target = 0;
        public double angle_from_target = 0;
        public double robot_x = 0;
        public double robot_y = 0;
        public double robot_angle;
        public boolean robot_auto_on = false;
        public double feet = .3048;
        public double angle = 0;
        public LLResult result;
        public void init() {
            limelight = hardwareMap.get(Limelight3A.class,"limelight");
            limelight.start();
            if(color == 0) {
                limelight.pipelineSwitch(4);
            }else{
                limelight.pipelineSwitch(5);
            }
            limelight.start();
        }

        public void camera_calculations(){
            result = limelight.getLatestResult();
            if(result.isValid()) {
                Pose3D botpose = result.getFiducialResults().get(0).getTargetPoseRobotSpace();
                x = botpose.getPosition().x - .06;
                y = botpose.getPosition().z;
                r = Math.sqrt(x * x + y * y);
                double B = Math.toRadians(180 - Math.toDegrees(angle));
                double other_angle = Math.toRadians(botpose.getOrientation().getPitch());
                angle = other_angle  - Math.atan(x/y);
                distance_from_target = Math.sqrt(r*r + tag_to_target_distance * tag_to_target_distance - 2 * r * tag_to_target_distance * Math.cos(B));
                angle_from_target = Math.toDegrees(Math.asin((r * sin(B)) / distance_from_target));
                robot_angle = 41 -  angle_from_target;

                robot_x = distance_from_target * Math.cos(Math.toRadians(robot_angle)) - 6 * feet;
                robot_y = 6 * feet - distance_from_target * sin(Math.toRadians(robot_angle));
                if(result.getFiducialResults().get(0).getFiducialId() == 24){
                    robot_y = (distance_from_target * Math.cos(Math.toRadians(robot_angle)) - 6 * feet) * -1;
                    robot_x = 6 * feet - distance_from_target * sin(Math.toRadians(robot_angle));
                }
                //robot_auto_on = ((robot_y >= robot_x && robot_y >= -robot_x && robot_y >= 0) || (abs(robot_y) * -1 <= robot_x - .8 && abs(robot_y) * -1 <= -robot_x - .8 && abs(robot_y) * -1 <= -.8)) && abs(follower.getVelocity().getMagnitude()) < robot_velocity_stop_shoot && gamepad1.right_trigger < .4 && turret.shootable;
            }else{
                robot_auto_on = false;
            }
        }
    }
    public static double close_flap = .23;
    public class shooter{
        public Servo flap;
        public DcMotorEx shoot1;
        public DcMotorEx shoot2;
        public double speed = 0;
        public double position = 0.04;
        public double speed_difference = 0;
        public double max_draw1 = 0;
        public double max_draw2 = 0;
        public double calc_velocity = 0;
        public double calc_rpm = 0;
        public double flap_velocity = 0;
        public double old_velocity = 0;
        public double last_shot = 1600;
        public boolean far_shooting = false;
        public void initialize(){
            flap = hardwareMap.get(Servo.class, "flap");
            shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
            shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
            PIDFCoefficients pidf = new PIDFCoefficients(50, 20, 0, 1);
            shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void shooting() {
            if (follower.getPose().getX() < 44 && teleop) {
                /*speed = speed_far;
                position = .6;
                far_shooting = true;*/
                speed = last_shot;
                position = flap_height;
            } else if (teleop) {
                far_shooting = false;
                if (dead_distance * .0254 < moving_offset) {
                    position = close_flap;
                    speed += flap_switch;
                } else {
                    position = flap_height;
                }
            }
            shoot2.setVelocity(speed);
            shoot1.setVelocity(speed);
            speed_difference = abs(shoot1.getVelocity() - speed);
            if(dead_distance * .0254 < moving_offset) {
                flap.setPosition(position);
            }else{
                flap.setPosition(position + (speed_difference) / 100 * adjust);
            }
        }
    }
    /*public static double Aclose_dis = 1.3;
    public static double Amid_dis = 2.4;
    public static double Afar_pos = 0;
    public static double Aclose_pos = .07;
    public static double Amid_pos = .016;
    public static double Aclose_shot_change = 50;
    public static double Amid_shot_change = 0;
    public static double Afar_shot_change = 200;
    public static double Auto_power = 1360;
    public static double Auto_angle = .02;
    public class shooter{
        public Servo flap;
        public DcMotorEx shoot1;
        public DcMotorEx shoot2;
        public double speed = 0;
        public double position = 0.04;
        public double speed_difference = 0;
        public double max_draw1 = 0;
        public double max_draw2 = 0;
        public double calc_velocity = 0;
        public double calc_rpm = 0;
        public double flap_velocity = 0;
        public double old_velocity = 0;
        public double last_shot = 0;
        public boolean far_shooting = false;
        public double shot_change = 0;
        public double rounded_speed = speed;
        public double speed_correct = 0;

        public void initialize(){
            flap = hardwareMap.get(Servo.class, "flap");
            shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
            shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
            PIDFCoefficients pidf = new PIDFCoefficients(150, 1, .001, 7);
            shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
            shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void shooting() {
             if(dead_distance * .0254 < Aclose_dis){
                position = Aclose_pos;
                shot_change = Aclose_shot_change;
            }else if(dead_distance * .0254 > Amid_dis){
                 position = Afar_pos;
                 shot_change = Afar_shot_change;
            }else{
                 position = Amid_pos;
                 shot_change = 0;
            }
            if(!teleop){
                position = Auto_angle;
                speed = Auto_power;
                shot_change = 0;
            }
            rounded_speed = Math.round(speed / 20) * 20 + shot_change;
            shoot2.setVelocity(rounded_speed);
            shoot1.setVelocity(rounded_speed);
            telemetry.addData("velocity", shoot1.getVelocity());
            flap.setPosition(position + .02);
        }
    }*/


    public class turret{
        public PIDController controller;

        public double p = .01, i = 0, d = .001;
        public double f = 0;
        public DcMotorEx turret;
        public double current_angle;
        public double limit = 100;
        public double turret_angle;
        public double power;
        public boolean zero = false;
        public boolean shootable = false;
        public double turret_target = 0;
        public double manual_angle = 0;
        public Servo turret_servo_1;
        public Servo turret_servo_2;
        public double turret_pos = 0;
        public double analog_offset = 0;
        public double servo_degrees = 90/.29;
        public double max = 25;
        public double multiplier = 1.2;
        public double a_slow = 1.2;
        public double angle_mod = .9;
        public double target_angle = .5;
        public boolean on_limit = false;
        public void initialize(){
            turret_servo_1 = hardwareMap.get(Servo.class, "turret_servo_1");
            turret_servo_2 = hardwareMap.get(Servo.class, "turret_servo_2");
            turret_servo_1.setDirection(Servo.Direction.REVERSE);
            turret_servo_2.setDirection(Servo.Direction.REVERSE);
        }
        public void turret_move(){
            dead_wheel_calculations();
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            if(zero){
                target_angle = manual_angle / servo_degrees + .5;
            }else{
                double angle = Math.toDegrees(location.getHeading());
                if (angle > 180) {
                    angle = 360 - angle;
                }else{
                    angle *= -1;
                }
                angle = dead_angle - angle;
                telemetry.addData("angle of turret", angle);
                target_angle = (angle) / servo_degrees + .5;
            }
            zero = false;
            if((target_angle - .5) * servo_degrees > limit){
                target_angle = (limit - 2) / servo_degrees + .5;
                on_limit = true;
            }else if((target_angle - .5) * servo_degrees < -limit - 78){
                target_angle = ((-limit - 78) + 2) / servo_degrees + .5;
                on_limit = true;
            }else{
                on_limit = false;
            }
            if(manual_turret){
                target_angle = .5;
            }
            turret_servo_1.setPosition(target_angle - .001);
            turret_servo_2.setPosition(target_angle + .001);


        }
    }
    public double dead_angle = 0;
    public double dead_distance = 0;
    public static double y_mod = -12;
    public static double x_mod = 2;
    public static double speed_shoot = 40;
    public Pose location = new Pose();
    public double calc_angle = 0;
    public double calc_distance = 0;
    public void dead_wheel_calculations(){
        if(color == 0){
            location = new Pose((130 + x_mod) - (follower.getPose().getX() - 3 * sin(follower.getPose().getHeading() + Math.toRadians(180))), (52.2 + y_mod) - (follower.getPose().getY() + 3 * cos(follower.getPose().getHeading() + Math.toRadians(180))), follower.getPose().getHeading() + Math.toRadians(180));

        }else{
            location = new Pose((130 + x_mod) - (follower.getPose().getX() + 3 * sin(follower.getPose().getHeading() + Math.toRadians(180))), (52.2 + y_mod) + (follower.getPose().getY() - 3 * cos(follower.getPose().getHeading() + Math.toRadians(180))), follower.getPose().getHeading() + Math.toRadians(180));

        }
        double old_angle = calc_angle;
        double old_distance = calc_distance;
        if(location.getX() <= 0){
            dead_angle = 90;
        }else{
            dead_angle = Math.toDegrees(Math.atan(location.getY() / location.getX()));
        }
        if(color == 0){
            dead_angle *= -1;
        }
        dead_distance = Math.sqrt(Math.pow(location.getX(), 2) + Math.pow(location.getY(), 2));
        telemetry.addData("distance", dead_distance * .0254);
        telemetry.addData("x", location.getX());
        telemetry.addData("y", location.getY());
        telemetry.addData("heading", follower.getPose().getHeading());

    }
    public static double spinpower = -.7;
    public class sensors{
        public DigitalChannel apin0;
        public DigitalChannel apin2;
        public boolean auto_outtake = false;
        public ElapsedTime intake_offset = new ElapsedTime();
        public ElapsedTime shoot_time_offset = new ElapsedTime();
        public RevColorSensorV3 colorfront1;
        public RevColorSensorV3 colorfront2;
        public RevColorSensorV3 colorback1;
        public double ball_counted = 0;
        public double sorts = 0;
        public RevColorSensorV3 colorback2;
        public DigitalChannel countfront;
        public DigitalChannel countback;
        public double front1 = 0;
        public double front2 = 0;
        public double back1 = 0;
        public double back2 = 0;
        public boolean sorted = false;
        public double sort_step = 0;
        public ElapsedTime sort_time = new ElapsedTime();
        public boolean middleisgreen = false;
        public boolean spin = false;
        public double sort_places = 0;
        public boolean start_shoot = false;
        Pose hold = null;
        public boolean auto_shoot = false;
        public boolean ball_in_intake = false;
        public void initialize(){
            apin0 = hardwareMap.get(DigitalChannel.class, "apin1");
            apin2 = hardwareMap.get(DigitalChannel.class, "apin3");
            colorfront1 = hardwareMap.get(RevColorSensorV3.class, "colorfront1");
            colorfront2 = hardwareMap.get(RevColorSensorV3.class, "colorfront2");
            colorback1 = hardwareMap.get(RevColorSensorV3.class, "colorback1");
            colorback2 = hardwareMap.get(RevColorSensorV3.class, "colorback2");
            countfront = hardwareMap.get(DigitalChannel.class, "flapsensefront");
            countback = hardwareMap.get(DigitalChannel.class, "flapsenseback");
            balls.clear();
        }
        public void sense(){
            /*if(balls.size() >= 3){
                JamLight.setPosition(1);
            }else{
                JamLight.setPosition(0);
            }*/
            count();
            front1 = colorfront1.getDistance(DistanceUnit.MM);
            front2 = colorfront2.getDistance(DistanceUnit.MM);
            back1 = colorback1.getDistance(DistanceUnit.MM);
            back2 = colorback2.getDistance(DistanceUnit.MM);
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            /*if(color == 0) {
                if (dead_distance * .0254 < 2.5 && dead_distance * .0254 > 1.2 && x > y + 48 + 14.5 && x > -y + 48 - 14.5 && !turret.on_limit && abs(follower.getVelocity().getMagnitude()) < 30) {
                    auto_shoot = true;
                }else{
                    auto_shoot = false;
                }
            }else{
                if (dead_distance * .0254 < 2.5 && dead_distance * .0254 > 1.2 && x > y + 48 + 14.5 && x > -y + 48 - 14.5 && !turret.on_limit && abs(follower.getVelocity().getMagnitude()) < 30) {
                    auto_shoot = true;
                }else{
                    auto_shoot = false;
                }
            }
            if(auto_shoot && gamepad1.left_trigger < .4){
                balls.clear();
                shooter.last_shot = shooter.speed;
                flippy_pos = flippy_up;
                spindexer.setPower(spin_speed);
                if (apin0.getState()) {
                    robot_going_forward = true;
                    intake.setPower(1);
                }else {
                    intake.setPower(gamepad1.right_trigger);
                }
                start_shoot = true;

            }else {*/
            if (gamepad1.left_trigger > .4) {
                balls.clear();
                shooter.last_shot = shooter.speed;
                flippy_pos = flippy_up;
                spindexer.setPower(spin_speed);
                start_shoot = true;

            } else if (balls.size() >= 3) {
                start_shoot = false;
                intake.setPower(-.3);
                sort();
            } else {
                if (apin0.getState()) {
                    robot_going_forward = true;
                    intake.setPower(1);
                    spindexer.setPower(-1);
                } else if (gamepad1.right_trigger > .2) {
                    intake.setPower(gamepad1.right_trigger);
                    spindexer.setPower(-gamepad1.right_trigger);
                } else {
                    if (colorfront() && balls.size() < 3) {
                        spindexer.setPower(-1);
                    } else {
                        spindexer.setPower(0);
                    }
                    intake.setPower(gamepad1.right_trigger);
                }
                start_shoot = false;
                sorted = false;
                sort_step = 0;
                moving_steps = 0;
                flippy_pos = flippy_hold;
            }


        }
        public ElapsedTime spit_time = new ElapsedTime();
        public boolean init_count = true;
        public void count(){
            if(init_count){
                spit_time.reset();
                init_count = false;
            }else {
                if (!colorfront()) {
                    spit_time.reset();
                } else if (spit_time.milliseconds() > 400) {
                    balls.clear();
                    gamepad1.rumble(500);
                    balls.add(1.0);
                    balls.add(1.0);
                    balls.add(1.0);
                }
            }
        }
        public void sort(){
            if(!sorted){
                if(sort_step == 0){
                    gamepad1.rumble(1000);
                    sort_time.reset();
                    if(!gamepad1.a && gamepad1.left_trigger < .4 && index_reverse) {
                        intake.setPower(-.5);
                    }
                    flippy_pos = flippy_hold;
                    sort_step = 1;
                }else if(sort_step == 1 && sort_time.milliseconds() >= 500){
                    if(firstisgreen() || lastisgreen()){
                        middleisgreen = false;
                    }else{
                        middleisgreen = true;
                    }
                    if(pattern == 1){
                        pattern1sort();
                    }else if(pattern == 2){
                        pattern2sort();
                    }else{
                        pattern3sort();
                    }
                    sort_step = 2;
                }else if(sort_step == 2){
                    if(sort_places == 0 || sorts == 2){
                        sort_step = 0;
                        sorts = 0;
                        sorted = true;
                        if(teleop) {
                            intake.setPower(0);
                        }
                        sort_time.reset();
                    }else if(sort_places == 1){
                        sort_once();
                    }else if(sort_places == 2){
                        sort_twice();
                    }
                }else if(sort_step == 3 && sort_time.milliseconds() >= 500){
                    sort_step = 1;
                    sorts += 1;
                    moving_steps = 0;
                }

            }else if(balls.size() >= 3){
                spindexer.setPower(0);
                flippy_pos = flippy_hold;
            }else if(balls.size() < 3){
                sorted = false;
                sort_step = 0;
                moving_steps = 0;
                sorts = 0;
            }
        }
        public double moving_steps = 0;
        public void sort_once(){
            if(moving_steps == 0){
                sort_time.reset();
                flippy_pos = flippy_down;
                spindexer.setPower(0);
                moving_steps = .5;
            }if(moving_steps == .5 && sort_time.milliseconds()> 100){
                moving_steps = 1;
                spindexer.setPower(-1);
                sort_time.reset();
            }
            if(moving_steps == 1 && sort_time.milliseconds() > 100){
                flippy_pos = flippy_hold;
                sort_time.reset();
                sort_step = 3;
            }
        }
        public void sort_twice(){
            if(moving_steps == 0){
                sort_time.reset();
                flippy_pos = flippy_down;
                spindexer.setPower(0);
                moving_steps = .5;
            }if(moving_steps == .5 && sort_time.milliseconds()> 100){
                moving_steps = 1;
                spindexer.setPower(-1);
                sort_time.reset();
            }if(moving_steps == 1 && sort_time.milliseconds() > 260){
                flippy_pos = flippy_hold;
                moving_steps = 2;
                sort_time.reset();
                sort_step = 3;
            }
        }
        public void pattern1sort(){
            if(firstisgreen()){
                sort_places = 0;
            }else if(middleisgreen){
                sort_places = 2;
            }else if(lastisgreen()){
                sort_places = 1;
            }
        }
        public void pattern2sort(){
            if(middleisgreen){
                sort_places = 0;
            }else if(lastisgreen()){
                sort_places = 2;
            }else if(firstisgreen()){
                sort_places = 1;
            }
        }
        public void pattern3sort(){
            if(lastisgreen()){
                sort_places = 0;
            }else if(firstisgreen()){
                sort_places = 2;
            }else if(middleisgreen){
                sort_places = 1;
            }
        }

        public void auto_index(){

        }
        public boolean last_time(){
            if(gamepad1.left_trigger < .4 || colorfront()){
                sensed = false;
            }
            if(!colorfront() && !sensed){
                sensed = true;
                shoot_time_offset.reset();
            }
            if(sensed && shoot_time_offset.milliseconds() > shot_time){
                return true;
            }else{
                return false;
            }

        }
        public boolean firstisgreen(){
            if(front1 < front2){
                if(colorfront1.getNormalizedColors().green > colorfront1.getNormalizedColors().blue){
                    return true;
                }else{
                    return false;
                }
            }else{
                if(colorfront2.getNormalizedColors().green > colorfront2.getNormalizedColors().blue){
                    return true;
                }else{
                    return false;
                }
            }
        }
        public boolean sensed = false;
        public boolean lastisgreen(){
            if(back1 < back2){
                if(colorback1.getNormalizedColors().green > colorback1.getNormalizedColors().blue){
                    return true;
                }else{
                    return false;
                }
            }else{
                if(colorback2.getNormalizedColors().green > colorback2.getNormalizedColors().blue){
                    return true;
                }else{
                    return false;
                }
            }
        }

        public boolean colorfront(){
            if(front2 < 140 || front1 < 140){
                return true;
            }else{
                return false;
            }

        }
        public boolean colorback(){
            if(back2 < 140 || back1 < 140){
                return true;
            }else{
                return false;
            }

        }
    }






}
