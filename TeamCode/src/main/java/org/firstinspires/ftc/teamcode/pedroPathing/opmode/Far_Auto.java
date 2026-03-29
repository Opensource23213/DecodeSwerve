package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Far_Auto", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class Far_Auto extends DecodeLibrary{
    public double forward = 0.25;
    public Path first_pick;
    public Path pick_after_stuff;
    public Path first_shoot;
    public Path second_shoot;
    public Path third_shoot;
    public Path fourth_shoot;
    public Path fifth_shoot;
    public Path second_pick;
    public Path third_pick;
    public Path park;
    public Path gate;
    public double shot = 0;
    public double old_turns = 0;
    public double steps = 0;
    public ElapsedTime gate_open = new ElapsedTime();
    public ElapsedTime shooting_time = new ElapsedTime();
    public sorting sorting = new sorting();
    public double sort_pos = 0;
    public double old_pattern = 2;
    public double old_color = 1;
    public boolean shooting = false;
    public double index_steps = 0;
    @Override
    public void init(){
        color = 1;
        teleop = false;
        if(color == 1){
            auto_pose = new Pose(0,.375, Math.toRadians(-90));
        }else{
            auto_pose = new Pose(0,-.375, Math.toRadians(90));
        }
        initialize();
        follower.setPose(auto_pose);
        cameraCode.limelight.pipelineSwitch(6);
        if(color == 0) {
            blue_init();
            angle_offset = -90;
        }else{
            red_init();
            angle_offset = 90;
        }
        tele_offset = turret.current_angle;
        pattern = 2;
        follower.setMaxPower(1);
        follower.setMaxPowerScaling(1);
        follower.followPath(first_shoot);
        sorting.sorted = true;
        turret.manual_angle = 0;
    }
    @Override
    public void init_loop(){
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        follower.drivetrain.setYVelocity(0);
        follower.update();
        if(gamepad1.right_bumper) {
            turret.manual_angle = -90;
            color = 0;
        }else if(gamepad1.left_bumper){
            color = 1;
            turret.manual_angle = 90;
        }
        turret.zero = true;
        turret.turret_move();
        sorting.sort_balls = true;
        cameraCode.camera_calculations();
        if(cameraCode.result.isValid()) {
            if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 23) {
                pattern = 3;
            } else if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 21) {
                pattern = 1;
            }else if(cameraCode.result.getFiducialResults().get(0).getFiducialId() == 22){
                pattern = 2;
            }
        }
        if(old_color != color || gamepad1.a) {
            initialize();
            if(color == 1){
                auto_pose = new Pose(0,.375, Math.toRadians(-90));
            }else{
                auto_pose = new Pose(0,-.375, Math.toRadians(90));
            }
            follower.setMaxPower(1);
            follower.setMaxPowerScaling(1);
            follower.setPose(auto_pose);
            cameraCode.limelight.pipelineSwitch(6);
            if (color == 0) {
                blue_init();
                old_color = 0;
            } else {
                red_init();
                old_color = 1;
            }

            follower.followPath(first_shoot);
        }
        flippy.setPosition(flippy_up);
        auto_pose = follower.getPose();
        telemetry.addData("color", color);
        telemetry.addData("pattern", pattern);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void loop() {
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        follower.update();
        auto_pose = follower.getPose();
        flippy.setPosition(flippy_pos);
        shooter.shooting();
        turret.turret_move();
        sorting.sort();
        shoot();
        if(follower.atParametricEnd() || !follower.isBusy() || follower.isRobotStuck()) {
            if(forward == .25){
                shoot();
                shooting = true;
                intake.setPower(0);
                if(steps == 2) {
                    shooting = false;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    follower.followPath(third_pick);
                    forward = .5;
                    steps = 0;
                }
            }else if(forward == .5){
                sorting.sorted = true;
                follower.followPath(fourth_shoot);
                forward = .75;
            }else if (forward == .75) {
                shoot();
                shooting = true;
                intake.setPower(0);
                if(steps == 2){
                    shooting = false;
                    robot_going_forward = true;
                    spindexer.setPower(-1);
                    follower.followPath(first_pick);
                    intake.setPower(1);
                    forward = .8;
                    steps = 0;
                }
            }else if (forward == .8) {
                intake.setPower(1);
                follower.followPath(gate);
                forward = 1;
            }else if(forward == 1 && (!follower.isTurning() || follower.isRobotStuck())){
                if(steps == 0){
                    gate_open.reset();
                    intake.setPower(-.4);
                    steps = 1;
                }else if(steps == 1 && gate_open.milliseconds() > 800) {
                    robot_going_forward = true;
                    sorting.sort_balls = true;
                    follower.followPath(second_shoot);
                    forward = 2;
                    steps = 0;
                }
            }else if(forward == 2){
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    follower.followPath(second_pick);
                    forward = 3;
                    steps = 0;
                }
            }else if(forward == 3){
                sorting.sorted = false;
                sorting.sort_balls = true;
                follower.followPath(third_shoot);
                forward = 6;
            }
            else if(forward == 6){
                intake.setPower(0);
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    index_reverse = true;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    follower.followPath(pick_after_stuff);
                    forward = 7;
                    steps = 0;
                }
            }else if(forward == 7){
                sorting.sort_balls = true;
                follower.followPath(fifth_shoot);
                forward = 8.1;
            }else if(forward == 8.1){
                intake.setPower(0);
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    follower.followPath(park);
                    forward = 9;
                    steps = 0;
                }
            }
        }
    }

    public void red_init(){
        first_shoot = new Path(new BezierCurve(new Pose(0,.375), new Pose(66, 0)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
        first_pick = new Path(new BezierCurve(new Pose(65,0), new Pose(47, -3), new Pose(46, -44)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        gate = new Path(new BezierCurve(first_pick.getLastControlPoint(), new Pose(55, -33), new Pose(56, -41)));
        gate.setConstantHeadingInterpolation(Math.toRadians(-190));
        second_shoot = new Path(new BezierCurve(gate.getLastControlPoint(), new Pose(56, -15), new Pose(72, -4)));
        second_shoot.setLinearHeadingInterpolation(Math.toRadians(-180),Math.toRadians(-80));
        second_pick = new Path(new BezierLine(second_shoot.getLastControlPoint(), new Pose(74, -33.5)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), new Pose(72, -10)));
        third_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
        third_pick = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(27, 0), new Pose(25, -43)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        fourth_shoot = new Path(new BezierCurve(third_pick.getLastControlPoint(), new Pose(35, 0), new Pose(72, -4)));
        fourth_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
        pick_after_stuff = new Path(new BezierCurve(new Pose(72, -4), new Pose(61,-45), new Pose(21, -50), new Pose(4, -49)));
        fifth_shoot = new Path(new BezierLine(pick_after_stuff.getLastControlPoint(), new Pose(72, -4)));
        fifth_shoot.setLinearHeadingInterpolation(Math.toRadians(-180),Math.toRadians(-90));
        park = new Path(new BezierLine(fourth_shoot.getLastControlPoint(), new Pose(72,-15)));
        park.setConstantHeadingInterpolation(Math.toRadians(-45));
    }
    public void blue_init(){
        first_shoot = new Path(new BezierCurve(new Pose(0,-.375), new Pose(66, 0)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        first_pick = new Path(new BezierCurve(new Pose(65,0), new Pose(53, 3), new Pose(51, 42)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        gate = new Path(new BezierCurve(first_pick.getLastControlPoint(), new Pose(53, 40), new Pose(56, 41)));
        gate.setConstantHeadingInterpolation(Math.toRadians(190));
        second_shoot = new Path(new BezierCurve(gate.getLastControlPoint(), new Pose(56, 15), new Pose(72, 4)));
        second_shoot.setConstantHeadingInterpolation(Math.toRadians(180));
        second_pick = new Path(new BezierLine(second_shoot.getLastControlPoint(), new Pose(74, 33.5)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), new Pose(72, 10)));
        third_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        third_pick = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(27, 0), new Pose(25, 43)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        fourth_shoot = new Path(new BezierCurve(third_pick.getLastControlPoint(), new Pose(35, 0), new Pose(72, 4)));
        fourth_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        pick_after_stuff = new Path(new BezierCurve(new Pose(72, 4), new Pose(52,45), new Pose(21, 50), new Pose(4, 50)));
        pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(180));
        fifth_shoot = new Path(new BezierLine(pick_after_stuff.getLastControlPoint(), new Pose(72, 4)));
        fifth_shoot.setConstantHeadingInterpolation(Math.toRadians(180));
        park = new Path(new BezierLine(fourth_shoot.getLastControlPoint(), new Pose(72,15)));
        park.setConstantHeadingInterpolation(Math.toRadians(180));
    }
    public void shoot(){
        if(shooting){
            intake.setPower(-1);
            shooter.speed = 1710;
            if(steps == 0 && sorting.sorted){
                shooting_time.reset();
                steps = .5;
            }else if(steps == .5 && shooting_time.milliseconds() > 150){
                shooting_time.reset();
                steps = 1;
                spindexer.setPower(1);
                flippy_pos = flippy_up;
            }else if(steps == 1 && shooting_time.milliseconds() > 1000){
                steps = 2;
                sorting.sorted = false;
            }
        }else{
            shooter.speed = 1710;
        }
    }

    public class sorting {
        public boolean colorfront() {
            front1 = sensors.colorfront1.getDistance(DistanceUnit.MM);
            front2 = sensors.colorfront2.getDistance(DistanceUnit.MM);
            if (front2 < 140 || front1 < 140) {
                return true;
            } else {
                return false;
            }

        }
        public double back1 = 0;
        public double back2 = 0;
        public double front1 = 0;
        public double front2 = 0;
        public boolean colorback(){
            back1 = sensors.colorback1.getDistance(DistanceUnit.MM);
            back2 = sensors.colorback2.getDistance(DistanceUnit.MM);
            if(back2 < 140 || back1 < 140){
                return true;
            }else{
                return false;
            }

        }
        public boolean sort_balls = false;

        public void sort() {
            if (sort_balls) {
                balls.clear();
                balls.add(1.0);
                balls.add(1.0);
                balls.add(1.0);
                if (sorting.sorted) {
                    sort_balls = false;
                } else {
                    sorter();
                }
            } else {
                flippy_pos = flippy_up;
            }
        }
        public double sort_step = 0;
        public boolean sorted = false;
        public ElapsedTime sort_time = new ElapsedTime();
        public boolean middleisgreen = false;
        public void sorter(){
            if(!sorted){
                if(sort_step == 0){
                    gamepad1.rumble(1000);
                    sort_time.reset();
                    if(index_reverse) {
                        intake.setPower(-.2);
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

            }

        }
        public double sorts = 0;
        public double sort_places = 0;
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
        public boolean firstisgreen(){
            if(front1 < front2){
                if(sensors.colorfront1.getNormalizedColors().green > sensors.colorfront1.getNormalizedColors().blue){
                    return true;
                }else{
                    return false;
                }
            }else{
                if(sensors.colorfront2.getNormalizedColors().green > sensors.colorfront2.getNormalizedColors().blue){
                    return true;
                }else{
                    return false;
                }
            }
        }
        public boolean sensed = false;
        public boolean lastisgreen(){
            if(back1 < back2){
                if(sensors.colorback1.getNormalizedColors().green > sensors.colorback1.getNormalizedColors().blue){
                    return true;
                }else{
                    return false;
                }
            }else{
                if(sensors.colorback2.getNormalizedColors().green > sensors.colorback2.getNormalizedColors().blue){
                    return true;
                }else{
                    return false;
                }
            }
        }
    }


}
