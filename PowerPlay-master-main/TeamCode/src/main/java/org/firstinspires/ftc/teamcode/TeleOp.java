package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Horizontal;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOP😘", group = "TEST")

public class TeleOp extends LinearOpMode {


    ElapsedTime timer = new ElapsedTime();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private double lastError = 0;
    private double integralSum = 0;
boolean transfer=false;
    public static double Kp = 0.01;
    public static double KpLeft = 0.01;

    boolean slowForwards, slowBackwards;

    double forward, strafe, rotate;
    double  slowRotateRight, slowRotateLeft ;

    public static double targetPosition = 0;
    public Horizontal intake = new Horizontal();
    private Drivetrain drive = new Drivetrain();
    public Lift vertical = new Lift();

    enum State {
        INIT,
        TRUE,
        FALSE
    }


    public void runOpMode() throws InterruptedException {


        boolean ReadyToTransfer =false;
        boolean ready= false;
        boolean stacking=false;
        boolean bagal=false;
        boolean cone=false;
        State searchingCone = State.FALSE;
        ElapsedTime transferTime = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime scoreCone = new ElapsedTime();
        ElapsedTime closeClaw = new ElapsedTime();
        boolean doublescore =false;
        ElapsedTime option = new ElapsedTime();


        drive.initDrivetrain(hardwareMap);
        vertical.initLiftTeleOp(hardwareMap);
        intake.initIntake(hardwareMap);

        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        targetPosition=0;


        vertical.AlignerServo.setPosition(0.8);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            dashboard.sendTelemetryPacket(packet);

            double powerRight = returnPower(targetPosition, vertical.liftRight.getCurrentPosition(),Kp,0,0);
            double powerLeft = returnPower(targetPosition, vertical.liftLeft.getCurrentPosition(), KpLeft, 0 ,0);

            vertical.liftRight.setPower(powerRight);
            vertical.liftLeft.setPower(powerLeft);

            forward = gamepad1.right_stick_y;
            strafe = -gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            slowRotateRight = gamepad1.right_trigger;
            slowRotateLeft = gamepad1.left_trigger;
            slowBackwards = gamepad1.left_bumper;
            slowForwards = gamepad1.right_bumper;

            if (slowBackwards) {
                forward = 0.2;
            } else if (slowForwards) {
                forward = -0.2;
            } else {
                forward = gamepad1.right_stick_y;
            }

            if (slowRotateRight > 0)
                rotate = slowRotateRight * 0.2;
            else if (slowRotateLeft > 0)
                rotate = slowRotateLeft * -0.2;
            else
                rotate = gamepad1.right_stick_x;

            drive.LR.setPower(-forward + strafe + rotate);
            drive.RR.setPower(forward + strafe + rotate);
            drive.RF.setPower(-forward + strafe - rotate);
            drive.LF.setPower(forward + strafe - rotate);

            if(gamepad2.right_bumper){ // lift extension
                intake.intakeExtend();
                searchingCone = State.TRUE;
            }


            if (gamepad1.dpad_right){

                intake.ArmRight.setPosition(0.41);
                intake.ArmLeft.setPosition(0.41);
                intake.UpAndDown.setPosition(0.215);
                intake.intakeClaw.setPosition(intake.opened);
                searchingCone=State.TRUE;

            }

            if (gamepad1.dpad_up){

                intake.ArmRight.setPosition(0.33);
                intake.ArmLeft.setPosition(0.33);
                intake.UpAndDown.setPosition(0.19);
                intake.intakeClaw.setPosition(intake.opened);
                searchingCone=State.TRUE;

            }
            if (gamepad1.dpad_left){


                intake.ArmLeft.setPosition(0.24);
                intake.ArmRight.setPosition(0.24);
                intake.UpAndDown.setPosition(0.15);
                intake.intakeClaw.setPosition(intake.opened);
                searchingCone=State.TRUE;
            }

            if (gamepad1.dpad_down){

                intake.ArmLeft.setPosition(0.15);
                intake.ArmRight.setPosition(0.15);
                intake.UpAndDown.setPosition(0.13);
                intake.intakeClaw.setPosition(intake.opened);
                searchingCone=State.TRUE;
            }


            if(gamepad2.dpad_right && searchingCone==State.TRUE){
                intake.intakeClaw.setPosition(0.3);
                searchingCone=State.FALSE;
                timer.reset();
                ready=true;
            }


            if(timer.milliseconds()>150 && searchingCone==State.FALSE && ready){
                intake.intakeRetractTeleOP();
                intake.UpAndDown.setPosition(0);
                transfer=false;

            }

            if(gamepad2.left_bumper && ready){ //starting transfer
                intake.intakeClaw.setPosition(0);
                transferTime.reset();
                ready=false;
                ReadyToTransfer=true;
            }

            if(transferTime.milliseconds()>250 && ReadyToTransfer){ //cone drop
                vertical.outTakeClaw.setPosition(vertical.closedClaw);
                ReadyToTransfer=false;
                bagal=true;
            }

            if(targetPosition!=0){
                intake.UpAndDown.setPosition(0.1);
            }



            if(gamepad2.dpad_left && bagal){
                vertical.outTakeRight.setPosition(0.8);
                vertical.outTakeLeft.setPosition(0.8);
                vertical.outTakeClaw.setPosition(0.9);
                vertical.AlignerServo.setPosition(0.8);
                bagal=false;

            }

           /* if(gamepad2.share){
                targetPosition=120;
                vertical.outTakeLeft.setPosition(0.75);
                vertical.outTakeRight.setPosition(0.75);
                intake.ArmLeft.setPosition(0.7);
                intake.ArmRight.setPosition(0.7);
                intake.UpAndDown.setPosition(0.20);
                intake.intakeClaw.setPosition(intake.opened);
        }*/


            if(gamepad1.y){
                intake.ArmRight.setPosition(0.08);
                intake.ArmLeft.setPosition(0.08);
                intake.intakeClaw.setPosition(intake.closed);
                searchingCone=State.TRUE;
            }
            if(gamepad1.a){
                intake.ArmRight.setPosition(0.075);
                intake.ArmLeft.setPosition(0.075);
                intake.UpAndDown.setPosition(0.1);

                searchingCone=State.TRUE;
            }
            if(gamepad1.b){
                intake.intakeClaw.setPosition(intake.opened);
                searchingCone =State.TRUE;
            }

            if(gamepad2.x){
                targetPosition=0;
                vertical.Low();
            }

            if(gamepad2.a){
                targetPosition = 0;
                vertical.AlignerServo.setPosition(0.8);
                scoreCone.reset();
                cone=false;
            }

            if(scoreCone.milliseconds()>150 && !cone){
                vertical.OutTakeDownTELEOP();
                cone=true;
            }

            if(gamepad1.options){
                searchingCone=State.TRUE;
                intake.ArmLeft.setPosition(0.5);
                intake.ArmRight.setPosition(0.5);
                intake.UpAndDown.setPosition(0.1);
            }



            if(gamepad2.b){
                targetPosition = 210;
                vertical.OutTakeUp();
                vertical.AlignerServo.setPosition(1);
            }

            if(gamepad1.touchpad){
                targetPosition=targetPosition+10;
            }

            if(gamepad2.y){
                targetPosition = 540;
                vertical.OutTakeUp();
                vertical.AlignerServo.setPosition(1);
                closeClaw.reset();
            }


            if(gamepad2.dpad_down && searchingCone==State.FALSE){
                searchingCone=State.TRUE;
                intake.ArmDownAll();
                intake.intakeClaw.setPosition(intake.opened);
            }

            if(gamepad2.touchpad && searchingCone==State.FALSE){
                searchingCone=State.TRUE;
                intake.ArmLeft.setPosition(0.5);
                intake.ArmRight.setPosition(0.5);
                intake.UpAndDown.setPosition(0.2);
            }

            packet.put("positionRight", vertical.liftRight.getCurrentPosition());
            packet.put("positionLeft", vertical.liftLeft.getCurrentPosition());
            packet.put("errorRight", lastError);
            packet.put("errorLeft", lastError);
            telemetry.addData("Timer:", timer);
            telemetry.update();

            dashboard.sendTelemetryPacket(packet);

        }

    }
    public double returnPower(double reference, double state, double Kp, double Kd, double Ki){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error- lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return  output;
    }
}