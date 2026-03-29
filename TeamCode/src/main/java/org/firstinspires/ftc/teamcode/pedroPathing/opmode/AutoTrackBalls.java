package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name="AutoTrackBalls", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class AutoTrackBalls extends DecodeLibrary{
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
    public boolean moved = false;
    public double x = 0;
    public double balls_seen = 0;
    public double start_x = 10;
    public  webcam cam = new webcam();
    @Override
    public void init(){
        color = 0;
        teleop = false;
        initialize();
        if(color == 1){
            auto_pose = new Pose(start_x,.375, Math.toRadians(-90));
        }else{
            auto_pose = new Pose(start_x,-.375, Math.toRadians(90));
        }
        blue_init();
        cam.initialize();
        follower.setPose(auto_pose);
        cameraCode.limelight.pipelineSwitch(1);
    }


    @Override
    public void loop() {
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        cam.look();
        double[] python = cameraCode.limelight.getLatestResult().getPythonOutput();
        if(!moved) {
            x = python[0];
            if(auto_pose.getX() + x < .5){
                x = -(auto_pose.getX() - .5);
            }
            balls_seen = python[1];
            follower.followPath(first_pick);
            follower.drivetrain.debugString();
        }
        follower.update();
        flippy_pos = flippy_hold;
        flippy.setPosition(flippy_hold);
        if(gamepad1.a && !moved){
            first_pick = new Path(new BezierCurve(auto_pose,new Pose(auto_pose.getX() + x, 18), new Pose(auto_pose.getX() + x, 47)));
            first_pick.setConstantHeadingInterpolation(Math.toRadians(90));
            follower.followPath(first_pick);
            intake.setPower(1);
            spindexer.setPower(-1);
            moved = true;
        }
        telemetry.addData("balls", balls_seen);
        telemetry.addData("x", x);
        telemetry.addData("tag", cam.tags);
        telemetry.update();
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
        pick_after_stuff = new Path(new BezierCurve(new Pose(72, 4), new Pose(61,45), new Pose(21, 50), new Pose(4, 50)));
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
    public class webcam{
        public AprilTagProcessor aprilTag;
        public List<Double> tags = new ArrayList<>();

        /**
         * The variable to store our instance of the vision portal.
         */
        public VisionPortal visionPortal;
        public void initialize(){
            aprilTag = AprilTagProcessor.easyCreateWithDefaults();
            visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), aprilTag);
        }
        public void look(){
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            tags.clear();
            for (AprilTagDetection detections : currentDetections){
                tags.add((double) detections.id);
            }
        }
    }


}
