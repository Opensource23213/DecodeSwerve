package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Near_Auto", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class Near_Auto extends DecodeLibrary{
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
    public boolean re_init = false;
    public Path nextPath = new Path();
    public boolean second_dump = false;
    @Override
    public void init(){
        color = 1;
        teleop = false;
        initialize();
        cameraCode.limelight.pipelineSwitch(6);
        if(color == 0) {
            follower.setPose(new Pose(122,25 - .375, Math.toRadians(180)));
            blue_init();
            angle_offset = -90;
            old_color = 0;
        }else{
            follower.setPose(new Pose(122,-25, Math.toRadians(-180)));
            red_init();
            angle_offset = 90;
            old_color = 1;
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
        cameraCode.camera_calculations();
        turret.turret_move();
        if (cameraCode.limelight.getLatestResult().getPipelineIndex() != 6) {
            cameraCode.limelight.pipelineSwitch(6);
        }
        turret.zero = true;
        if (color == 0) {
            turret.manual_angle = 54;
        } else {
            turret.manual_angle = -50;
        }
        if(gamepad1.a){
            color = 1;
            re_init = true;
        }else if(gamepad1.b){
            color = 0;
            re_init = true;
        }

        if (cameraCode.result.isValid()) {
            if (color == 0) {
                if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 21) {
                    pattern = 3;
                } else if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 22) {
                    pattern = 1;
                } else if(cameraCode.result.getFiducialResults().get(0).getFiducialId() == 23){
                    pattern = 2;
                }
            } else {
                if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 22) {
                    pattern = 3;
                } else if (cameraCode.result.getFiducialResults().get(0).getFiducialId() == 23) {
                    pattern = 1;
                } else if(cameraCode.result.getFiducialResults().get(0).getFiducialId() == 21){
                    pattern = 2;
                }
            }
        }
        if(old_color != color || re_init) {
            re_init = false;
            initialize();
            cameraCode.limelight.pipelineSwitch(6);
            if (color == 0) {
                follower.setPose(new Pose(122,25 - .375, Math.toRadians(180)));
                blue_init();
                angle_offset = -90;
                old_color = 0;
            } else {
                follower.setPose(new Pose(122,-24, Math.toRadians(-180)));
                red_init();
                angle_offset = 90;
                old_color = 1;
            }
            follower.followPath(first_shoot);
        }
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
        turret.zero = true;
        turret.manual_angle = 90;
        turret.turret_move();
        sorting.sort();
        shoot();
        if(follower.atParametricEnd() || !follower.isBusy() || follower.isRobotStuck() || close(follower.getPose(), follower.getCurrentPath()) || steps == 2 || (follower.getCurrentPath() == gate && abs(follower.getPose().getY()) > abs(gate.getLastControlPoint().getY()) - error)) {
            if(forward == .25){
                shooting = true;
                nextPath = first_pick;
                shoot();
                intake.setPower(0);
                if(steps == 2) {
                    shooting = false;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    forward = .5;
                    steps = 0;
                }
            }
            else if(forward == .5){
                follower.followPath(second_shoot);
                forward = 1;
            }
            else  if(forward == 1){
                shooting = true;
                nextPath = second_pick;
                shoot();
                intake.setPower(0);
                if(steps == 2) {
                    shooting = false;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    follower.followPath(second_pick);
                    forward = 2;
                    steps = 0;
                }
            }
            else if(forward == 2){
                follower.followPath(third_shoot);
                forward = 3;
            }else if(forward == 3){
                shooting = true;
                nextPath = third_pick;
                shoot();
                intake.setPower(0);
                if(steps == 2) {
                    shooting = false;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    follower.followPath(gate);
                    forward = 4;
                    steps = 0;
                }
            }else if(forward == 4){
                follower.followPath(third_pick);
                forward = 4.5;
            }
            else if(forward == 4.5){
                follower.followPath(fourth_shoot);
                second_dump = false;
                forward = 5;
            }
            else if(forward == 5){
                shooting = true;
                nextPath = pick_after_stuff;
                shoot();
                intake.setPower(0);
                if(steps == 2) {
                    shooting = false;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    forward = 6;
                    steps = 0;
                }
            }else if(forward == 6){
                follower.followPath(fifth_shoot);
                forward = 7;
            }else if(forward == 7){
                if(!shooting){
                    pick_after_stuff = new Path(new BezierCurve(fourth_shoot.getLastControlPoint(), new Pose(2, -42)));
                    pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(-155));
                    fifth_shoot = new Path(new BezierLine(third_pick.getLastControlPoint(), fourth_shoot.getLastControlPoint()));
                    fifth_shoot.setConstantHeadingInterpolation(Math.toRadians(-155));

                }
                shooting = true;
                nextPath = pick_after_stuff;
                shoot();
                intake.setPower(0);
                if(steps == 2) {
                    shooting = false;
                    spindexer.setPower(-1);
                    intake.setPower(1);
                    forward = 6;
                    steps = 0;
                }
            }
        }
    }

    public void red_init(){
        first_shoot = new Path(new BezierLine(new Pose(123,-24), new Pose(100, -32)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(-180));
        first_pick = new Path(new BezierLine(first_shoot.getLastControlPoint(), new Pose(74, -32)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(-180));
        second_shoot = new Path(new BezierLine(first_pick.getLastControlPoint(), new Pose(86, -32)));
        second_shoot.setConstantHeadingInterpolation(Math.toRadians(-180));
        second_pick = new Path(new BezierLine(second_shoot.getLastControlPoint(), new Pose(54, -32)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(-180));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), second_shoot.getLastControlPoint()));
        third_shoot.setConstantHeadingInterpolation(Math.toRadians(-180));
        gate = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(62, -32), new Pose(60, -39)));
        gate.setConstantHeadingInterpolation(Math.toRadians(-180));
        third_pick = new Path(new BezierLine(new Pose(43, -32), new Pose(33, -32)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(-180));
        fourth_shoot = new Path(new BezierLine(third_pick.getLastControlPoint(), new Pose(70, -9)));
        fourth_shoot.setConstantHeadingInterpolation(Math.toRadians(-180));
        pick_after_stuff = new Path(new BezierCurve(fourth_shoot.getLastControlPoint(), new Pose(63, -38), new Pose(61, -38), new Pose(2, -42)));
        pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(-180));
        fifth_shoot = new Path(new BezierLine(third_pick.getLastControlPoint(), fourth_shoot.getLastControlPoint()));
        fifth_shoot.setConstantHeadingInterpolation(Math.toRadians(-180));
        park = new Path(new BezierLine(fourth_shoot.getLastControlPoint(), new Pose(fourth_shoot.getLastControlPoint().getX(), fourth_shoot.getLastControlPoint().getY() + .1)));
        park.setConstantHeadingInterpolation(Math.toRadians(-155));
    }
    public void blue_init(){
        first_shoot = new Path(new BezierLine(new Pose(123,-24), new Pose(100, -44)));
        first_shoot.setConstantHeadingInterpolation(-180);
    }
    public void shoot(){
        if(shooting){
            if(steps == 0){
                follower.followPath(nextPath);
                shooting_time.reset();
                steps = 1;
                spindexer.setPower(1);
                flippy_pos = flippy_up;
            }else if(steps == 1 && shooting_time.milliseconds() > 900){
                steps = 2;
                sorting.sorted = false;
                sorting.sort_balls = false;
            }
            follower.drivetrain.setYVelocity(0);
            intake.setPower(-.4);
        }
    }
    public static double error = 2;
    public boolean close(Pose robot, Path path){
        boolean x_correct = abs(path.getLastControlPoint().getX() - robot.getX()) < error;
        boolean y_correct = abs(path.getLastControlPoint().getY() - robot.getY()) < error;
        return x_correct && y_correct;
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
