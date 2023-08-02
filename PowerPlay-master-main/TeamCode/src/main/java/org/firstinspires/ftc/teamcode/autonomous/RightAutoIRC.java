package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeleOp;
import org.firstinspires.ftc.teamcode.detection.SleeveDetectionRight;
import org.firstinspires.ftc.teamcode.odometry.util.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.odometry.util.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.odometry.util.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name ="RightHighICR", group = "Test")
public class RightAutoIRC extends LinearOpMode {

    SampleMecanumDrive drive;
    TeleOp teleop = new TeleOp();
    Pose2d startPose = new Pose2d(37, -64.5, Math.toRadians(90)); //35.5

    TrajectorySequence transformer;

    Trajectory placeTransformer;

    TrajectorySequence pos1;
    TrajectorySequence pos2;
    Trajectory traj3;
    Trajectory traj4;
    Trajectory traj5;
    Trajectory traj6;
    Trajectory traj7;
    Trajectory traj8;
    Trajectory traj9;
    Trajectory traj10;
    Trajectory traj11;
    Trajectory parcare;
    OpenCvCamera camera;

    Trajectory parcare1st;

    double turnValue = -78;




    private SleeveDetectionRight sleeveDetection;
    private String webcamName = "Webcam 1";
    public static double targetPosition = 0;

    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        teleop.vertical.initLiftAuto(hardwareMap);
        teleop.intake.initIntake(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetectionRight();
        camera.setPipeline(sleeveDetection);

        FtcDashboard.getInstance().startCameraStream(camera, 20);


        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted() && !isStopRequested()) {

            telemetry.addData("Case: ", sleeveDetection.getPosition());
            telemetry.update();


        }
        waitForStart();

        if(opModeIsActive() && !isStopRequested()) {

            if(sleeveDetection.getPosition()== SleeveDetectionRight.ParkingPosition.RIGHT){
                stacking(65, -14);
            }else if(sleeveDetection.getPosition()== SleeveDetectionRight.ParkingPosition.CENTER){
                stacking(40, -14);
            }else if(sleeveDetection.getPosition()== SleeveDetectionRight.ParkingPosition.LEFT){
                stacking(18, -18);
            }

        }

    }

    public void correctPose(Pose2d targetPose){
        Pose2d poseEstimate = drive.getPoseEstimate();
        if(poseEstimate.equals(targetPose)){
            return;
        }

        try {
            drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(poseEstimate).lineToLinearHeading(
                            new Pose2d(targetPose.getX() + 1E-10, targetPose.getY() + 1E-10, targetPose.getHeading() )
                    ).build()
            );
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    public void stacking(double x , double y){

        pos1 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(37.7, -5.28),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(82.5))
                .build();


        transformer = drive.trajectorySequenceBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(43, 21, Math.toRadians(174.5)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // placeTransformer = drive.trajectoryBuilder(transformer.end())
        //         .lineToLinearHeading(new Pose2d(-46, 12, Math.toRadians(-19.87)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
        //         .build();



        teleop.vertical.outTakeClaw.setPosition(teleop.vertical.closedClaw);
        drive.followTrajectorySequence(pos1);


        teleop.vertical.HighAuto();
        sleep(100);
        teleop.intake.intakeRight.setPosition(0.35);
        teleop.intake.intakeLeft.setPosition(0.35);
        teleop.vertical.OutTakeUpAuto();
        sleep(600);


        teleop.intake.intakeRight.setPosition(0.6);
        teleop.intake.intakeLeft.setPosition(0.6);
        teleop.intake.ArmRight.setPosition(0.44);
        teleop.intake.ArmLeft.setPosition(0.44);
        teleop.intake.UpAndDown.setPosition(0.215);
        teleop.intake.intakeClaw.setPosition(0.01);
        sleep(150);
        placeCone();
        sleep(300);

        collectFirst();
        sleep(150);
        teleop.vertical.outTakeClaw.setPosition(0.9);
        sleep(450);

        //TODO: CICLU 1
        downLiftAuto();
        grabCone();
        sleep(230);


        upArm();
        sleep(250);

        teleop.vertical.outTakeClaw.setPosition(0.854);
        teleop.vertical.outTakeLeft.setPosition(0);
        teleop.vertical.outTakeRight.setPosition(0);
        sleep(200);
        retract();
        sleep(350);
        teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
        sleep(300);
        teleop.vertical.outTakeClaw.setPosition(teleop.vertical.closedClaw);
        sleep(200);
        teleop.vertical.HighAuto();
        sleep(50);
        teleop.vertical.OutTakeUpAuto();


        teleop.intake.intakeRight.setPosition(0.6);
        teleop.intake.intakeLeft.setPosition(0.6);
        teleop.intake.ArmRight.setPosition(0.36);
        teleop.intake.ArmLeft.setPosition(0.36);
        teleop.intake.UpAndDown.setPosition(0.19);
        teleop.intake.intakeClaw.setPosition(0.01);

        sleep(330);
        placeCone();
        collectSecond();
        sleep(150);
        teleop.vertical.outTakeClaw.setPosition(0.9);
        sleep(300);

        //TODO: CICLU 2

        downLiftAuto();
        grabCone();
        sleep(240);

        upArm();
        sleep(240);

        teleop.vertical.outTakeClaw.setPosition(0.86);
        teleop.vertical.outTakeLeft.setPosition(0);
        teleop.vertical.outTakeRight.setPosition(0);
        sleep(200);
        retract();
        sleep(340);
        teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
        sleep(300);
        teleop.vertical.outTakeClaw.setPosition(teleop.vertical.closedClaw);
        sleep(200);




        teleop.intake.intakeRight.setPosition(0.6);
        teleop.intake.intakeLeft.setPosition(0.6);
        teleop.intake.ArmRight.setPosition(0.44);
        teleop.intake.ArmLeft.setPosition(0.44);
        teleop.intake.UpAndDown.setPosition(0.15);
        teleop.intake.intakeClaw.setPosition(0.01);

        teleop.vertical.HighAuto();
        sleep(50);
        teleop.vertical.OutTakeUpAuto();
        sleep(330);
        collectThird();
        placeCone();
        sleep(150);
        teleop.vertical.outTakeClaw.setPosition(0.9);
        sleep(250);

        //TODO: CICLU 3

        downLiftAuto();
        grabCone();
        sleep(250);

        upArm();
        sleep(240);

        teleop.vertical.outTakeClaw.setPosition(0.86);
        teleop.vertical.outTakeLeft.setPosition(0);
        teleop.vertical.outTakeRight.setPosition(0);
        sleep(200);
        retract();
        sleep(340);
        teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
        sleep(300);
        teleop.vertical.outTakeClaw.setPosition(teleop.vertical.closedClaw);
        sleep(200);

        teleop.intake.intakeRight.setPosition(0.6);
        teleop.intake.intakeLeft.setPosition(0.6);
        teleop.intake.ArmLeft.setPosition(0.26);
        teleop.intake.ArmRight.setPosition(0.26);
        teleop.intake.UpAndDown.setPosition(0.15);
        teleop.intake.intakeClaw.setPosition(0.01);

        teleop.vertical.HighAuto();
        sleep(50);
        teleop.vertical.OutTakeUpAuto();

        sleep(330);
        collectFourth();
        placeCone();
        sleep(150);
        teleop.vertical.outTakeClaw.setPosition(0.9);
        sleep(290);

        //TODO: CICLU 4

        downLiftAuto();
        grabCone();
        sleep(240);

        upArm();
        sleep(250);

        teleop.vertical.outTakeClaw.setPosition(0.86);
        teleop.vertical.outTakeLeft.setPosition(0);
        teleop.vertical.outTakeRight.setPosition(0);
        sleep(200);
        retract();
        sleep(340);
        teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
        sleep(300);
        teleop.vertical.outTakeClaw.setPosition(teleop.vertical.closedClaw);
        sleep(200);

        teleop.intake.intakeRight.setPosition(0.6);
        teleop.intake.intakeLeft.setPosition(0.6);
        teleop.intake.ArmLeft.setPosition(0.2);
        teleop.intake.ArmRight.setPosition(0.2);
        teleop.intake.UpAndDown.setPosition(0.13);
        teleop.intake.intakeClaw.setPosition(0.01);


        teleop.vertical.HighAuto();
        sleep(50);
        teleop.vertical.OutTakeUpAuto();
        sleep(330);
        collectFifth();
        placeCone();
        sleep(150);
        teleop.vertical.outTakeClaw.setPosition(0.9);
        sleep(290);

        //TODO: CICLU 5

        downLiftAuto();
        grabCone();
        sleep(270);
        upArm();
        sleep(250);

        teleop.vertical.outTakeClaw.setPosition(0.86);
        teleop.vertical.outTakeLeft.setPosition(0);
        teleop.vertical.outTakeRight.setPosition(0);
        sleep(200);
        retract();
        sleep(350);
        teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
        sleep(300);
        teleop.vertical.outTakeClaw.setPosition(teleop.vertical.closedClaw);
        sleep(200);


        teleop.vertical.HighAuto();
        sleep(50);
        teleop.vertical.OutTakeUpAuto();



        teleop.intake.intakeRight.setPosition(teleop.intake.retracted + 0.1);
        teleop.intake.intakeLeft.setPosition(teleop.intake.retracted + 0.1);
        teleop.intake.ArmRight.setPosition(0.36);
        teleop.intake.ArmLeft.setPosition(0.36);
        teleop.intake.UpAndDown.setPosition(0.19);
        teleop.intake.intakeClaw.setPosition(0.01);


        sleep(298);
        teleop.intake.intakeRight.setPosition(teleop.intake.retracted);
        teleop.intake.intakeLeft.setPosition(teleop.intake.retracted);
        teleop.intake.ArmLeft.setPosition(0.2);
        teleop.intake.ArmRight.setPosition(0.2);
        teleop.intake.UpAndDown.setPosition(0.13);
        teleop.intake.intakeClaw.setPosition(0.01);
        placeCone();
        sleep(150);
        teleop.vertical.outTakeClaw.setPosition(0.9);
        sleep(300);

        teleop.vertical.downLift();

        teleop.intake.intakeRight.setPosition(0.4);
        teleop.intake.intakeLeft.setPosition(0.4);

        drive.followTrajectorySequence(transformer);


        teleop.intake.intakeRight.setPosition(0.77);
        teleop.intake.intakeLeft.setPosition(0.77);
        teleop.intake.ArmLeft.setPosition(0.14);
        teleop.intake.ArmRight.setPosition(0.14);
        teleop.intake.UpAndDown.setPosition(0.115);
        teleop.intake.intakeClaw.setPosition(-1);

        sleep(430);

        grabCone();
        sleep(400);



        upArm();
        sleep(900);

        teleop.vertical.outTakeClaw.setPosition(0.86);
        targetPosition = 732;
        teleop.vertical.outTakeLeft.setPosition(0);
        teleop.vertical.outTakeRight.setPosition(0);
        sleep(250);
        retract();
        sleep(400);
//        drive.followTrajectorySequence(placeTransformer);
        teleop.intake.intakeClaw.setPosition(teleop.intake.opened);
        sleep(250);
        teleop.vertical.outTakeClaw.setPosition(teleop.vertical.closedClaw);


        HighAutoTransformer();
        sleep(50);
        teleop.vertical.OutTakeUpAuto();
        sleep(350);
        placeCone();
        sleep(100);
        teleop.vertical.outTakeClaw.setPosition(0.9);
        sleep(250);




        parkingRetract();
//        drive.followTrajectory(parcare);
        drive.update();
    }


    public void retract(){

        teleop.intake.intakeRight.setPosition(teleop.intake.retracted);
        teleop.intake.intakeLeft.setPosition(teleop.intake.retracted);

        teleop.intake. ArmLeft.setPosition(0.57);
        teleop.intake.ArmRight.setPosition(0.57);

        teleop.intake.UpAndDown.setPosition(0);
    }

    public void upArm (){
        teleop.intake.ArmRight.setPosition(teleop.intake.ArmUp);
        teleop.intake.ArmLeft.setPosition(teleop.intake.ArmUp);

        teleop.intake.UpAndDown.setPosition(0);
    }

    public void UpAndDown(){
        teleop.intake.UpAndDown.setPosition(0);
    }

    public void grabCone(){
        teleop.intake.intakeClaw.setPosition(teleop.intake.closed);
    }

    public void placeCone(){
        teleop.vertical.outTakeLeft.setPosition(0.88);
        teleop.vertical.outTakeRight.setPosition(0.88);
    }

    public void collectFirst(){

        teleop.intake.intakeRight.setPosition(0.805);
        teleop.intake.intakeLeft.setPosition(0.805);
        teleop.intake.ArmRight.setPosition(0.423);
        teleop.intake.ArmLeft.setPosition(0.423);
        teleop.intake.UpAndDown.setPosition(0.215);
        teleop.intake.intakeClaw.setPosition(0.01);
    }

    public void collectSecond(){
        //teleop.vertical.outTakeLeft.setPosition(1.02- teleop.vertical.outTakeDown);
        //teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
        teleop.intake.intakeRight.setPosition(0.767);
        teleop.intake.intakeLeft.setPosition(0.767);
        teleop.intake.ArmRight.setPosition(0.36);
        teleop.intake.ArmLeft.setPosition(0.36);
        teleop.intake.UpAndDown.setPosition(0.19);
        teleop.intake.intakeClaw.setPosition(0.01);
    }

    public void collectThird(){
        //teleop.vertical.outTakeLeft.setPosition(1.02 - teleop.vertical.outTakeDown);
        //teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
        teleop.intake.intakeRight.setPosition(0.75);
        teleop.intake.intakeLeft.setPosition(0.75);
        teleop.intake.ArmLeft.setPosition(0.27);
        teleop.intake.ArmRight.setPosition(0.27);
        teleop.intake.UpAndDown.setPosition(0.16);
        teleop.intake.intakeClaw.setPosition(0.01);
    }

    public void collectFourth(){
        //teleop.vertical.outTakeLeft.setPosition(1.02 - teleop.vertical.outTakeDown);
        //teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
        teleop.intake.intakeRight.setPosition(0.71);
        teleop.intake.intakeLeft.setPosition(0.71);
        teleop.intake.ArmLeft.setPosition(0.21);
        teleop.intake.ArmRight.setPosition(0.21);
        teleop.intake.UpAndDown.setPosition(0.13);
        teleop.intake.intakeClaw.setPosition(0.01);
    }

    public void collectFifth(){
        //teleop.vertical.outTakeLeft.setPosition(1.02 - teleop.vertical.outTakeDown);
        //teleop.vertical.outTakeRight.setPosition(teleop.vertical.outTakeDown);
        teleop.intake.intakeRight.setPosition(0.72);
        teleop.intake.intakeLeft.setPosition(0.72);
        teleop.intake.ArmLeft.setPosition(0.14);
        teleop.intake.ArmRight.setPosition(0.14);
        teleop.intake.UpAndDown.setPosition(0.1);
        teleop.intake.intakeClaw.setPosition(0.01);
    }

    public void parkingRetract(){
        teleop.vertical.downLift();
        teleop.intake.intakeLeft.setPosition(teleop.intake.retracted);
        teleop.intake.intakeRight.setPosition(teleop.intake.retracted);
        teleop.intake.ArmRight.setPosition(teleop.intake.ArmUp);
        teleop.intake.ArmLeft.setPosition(teleop.intake.ArmUp);
        teleop.intake.UpAndDown.setPosition(0);
    }


    public void downLiftAuto() {

        //MOTOARE

        teleop.vertical.liftLeft.setTargetPosition(0);
        teleop.vertical.liftRight.setTargetPosition(0);

        teleop.vertical.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        teleop.vertical.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        teleop.vertical.liftLeft.setPower(0.9);
        teleop.vertical.liftRight.setPower(0.9);
    }



    public void HighAutoTransformer() {

        teleop.vertical.outTakeClaw.setPosition(0.7);

        teleop.vertical.liftLeft.setTargetPosition(535);
        teleop.vertical.liftRight.setTargetPosition(535);

        teleop.vertical.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        teleop.vertical.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        teleop.vertical.liftLeft.setPower(1);
        teleop.vertical.liftRight.setPower(1);

    }


    public void telemetry(){
        telemetry.addData("Left:", teleop.vertical.liftLeft.getCurrentPosition());
        telemetry.addData("Right:", teleop.vertical.liftRight.getCurrentPosition());
        telemetry.addData("X", drive.getPoseEstimate().getX());
        telemetry.addData("Y", drive.getPoseEstimate().getY());
        telemetry.addData("Heading", drive.getPoseEstimate().getHeading());
        telemetry.update();
    }
}

