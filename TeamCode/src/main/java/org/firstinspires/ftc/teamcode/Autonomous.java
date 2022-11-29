//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//
//@Autonomous(name="A5auto", group="Pushbot")
//public class A5auto extends LinearOpMode {
//
//    public robotInit robot = new robotInit();
//    ElapsedTime runtime = new ElapsedTime();
//
//        public const int LENGTH_OF_ROBOT = 18; //inches
//        public const int WIDTH_OF_ROBOT = 15; //inches
//        public const double CIRCUMFERENCE = WIDTH_OF_ROBOT * 2 * Math.PI;
//        public const double INCH_PER_DEGREE = CIRCUMFERENCE / 360;
//        public const double DISTANCE_BETWEEN_THE_CLAW_AND_JUNCTION_IN_INCHES = 3.8;
//    @Override
//    public void runOpMode() {
//
//        robot.init(hardwareMap);
//
//        resetEncoder();
//        startEncoderMode();
//
//
//        // Wait for the game to start (driver presses PLAY)
//        telemetry.addLine("Waiting for start");
//        telemetry.update();
//        waitForStart();
//
//
//
//
//
//
//        // STEP 1 - Delivering duck on carousel
//      //  strafeRight(20);
//
//
//        moveForward(24-LENGTH_OF_ROBOT);
//        turnLeftDegree(45);
//
//        raise(20); // RANDOM number of COUNTS; MUST BE MAXIMUM
//        moveForward(DISTANCE_BETWEEN_THE_CLAW_AND_JUNCTION_IN_INCHES);
//        raise(-5);
//        openTheClaw();
//
//
//
//
//
//
//
//    }
//
//    public void openTheClaw(){
//        robot.closer.setPosition(.25);
//    }
//    public void closeTheClaw(){
//        robot.closer.setPosition(.5);
//    }
//
//    public void turnRightDegree(double degrees){
//       turnRight(INCH_PER_DEGREE * degrees);
//    }
//
//    public void turnLeftDegree(double degrees){
//        turnLeft(INCH_PER_DEGREE * degrees);
//    }
//
//    // FUNCTION TO TURN RIGHT
//    public void turnRight(double inches) {
//        int newmotorFLTarget;
//        int newmotorFRTarget;
//        int newmotorBLTarget;
//        int newmotorBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
//
//        robot.motorFL.setTargetPosition(newmotorFLTarget);
//        robot.motorFR.setTargetPosition(newmotorFRTarget);
//        robot.motorBL.setTargetPosition(newmotorBLTarget);
//        robot.motorBR.setTargetPosition(newmotorBRTarget);
//
//        // Turn On RUN_TO_POSITION
//        // robot moves to set position
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
//        runtime.reset();
//        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
//            // Display it for the driver.
//            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
//            telemetry.update();
//        }
//    }
//
//
//
//    // FUNCTION TO TURN LEFT
//    public void turnLeft(double inches) {
//        int newmotorFLTarget;
//        int newmotorFRTarget;
//        int newmotorBLTarget;
//        int newmotorBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
//
//        robot.motorFL.setTargetPosition(newmotorFLTarget);
//        robot.motorFR.setTargetPosition(newmotorFRTarget);
//        robot.motorBL.setTargetPosition(newmotorBLTarget);
//        robot.motorBR.setTargetPosition(newmotorBRTarget);
//
//        // Turn On RUN_TO_POSITION
//        // robot moves to set position
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
//        runtime.reset();
//        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
//            // Display it for the driver.
//            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
//            telemetry.update();
//        }
//    }
//
//
//
//    // FUNCTION TO STRAFE LEFT
//    public void strafeLeft(double inches) {
//        int newmotorFLTarget;
//        int newmotorFRTarget;
//        int newmotorBLTarget;
//        int newmotorBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
//
//        robot.motorFL.setTargetPosition(newmotorFLTarget);
//        robot.motorFR.setTargetPosition(newmotorFRTarget);
//        robot.motorBL.setTargetPosition(newmotorBLTarget);
//        robot.motorBR.setTargetPosition(newmotorBRTarget);
//
//        // Turn On RUN_TO_POSITION
//        // robot moves to set position
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
//        runtime.reset();
//        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
//            // Display it for the driver.
//            telemetry.addData("Strafing left", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
//            telemetry.update();
//        }
//    }
//
//
//
//
//    // FUNCTION TO STRAFE RIGHT
//    public void strafeRight(double inches) {
//        int newmotorFLTarget;
//        int newmotorFRTarget;
//        int newmotorBLTarget;
//        int newmotorBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
//
//        robot.motorFL.setTargetPosition(newmotorFLTarget);
//        robot.motorFR.setTargetPosition(newmotorFRTarget);
//        robot.motorBL.setTargetPosition(newmotorBLTarget);
//        robot.motorBR.setTargetPosition(newmotorBRTarget);
//
//        // Turn On RUN_TO_POSITION
//        // robot moves to set position
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
//        runtime.reset();
//        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
//            // Display it for the driver.
//            telemetry.addData("Strafing right", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
//            telemetry.update();
//        }
//    }
//
//
//
//
//
//    // FUNCTION TO MOVE BACKWARD
//    public void moveBackward(double inches) {
//        int newmotorFLTarget;
//        int newmotorFRTarget;
//        int newmotorBLTarget;
//        int newmotorBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
//
//        robot.motorFL.setTargetPosition(newmotorFLTarget);
//        robot.motorFR.setTargetPosition(newmotorFRTarget);
//        robot.motorBL.setTargetPosition(newmotorBLTarget);
//        robot.motorBR.setTargetPosition(newmotorBRTarget);
//
//        // Turn On RUN_TO_POSITION
//        // robot moves to set position
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
//        runtime.reset();
//        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
//            // Display it for the driver.
//            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
//            telemetry.update();
//        }
//    }
//
//
//    //hi
//    // FUNCTION TO MOVE FORWARD
//    public void moveForward(double inches) {
//        int newmotorFLTarget;
//        int newmotorFRTarget;
//        int newmotorBLTarget;
//        int newmotorBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
//        newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
//        robot.motorFL.setTargetPosition(newmotorFLTarget);
//        robot.motorFR.setTargetPosition(newmotorFRTarget);
//        robot.motorBL.setTargetPosition(newmotorBLTarget);
//        robot.motorBR.setTargetPosition(newmotorBRTarget);
//
//        // Turn On RUN_TO_POSITION
//        // robot moves to set position
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
//        runtime.reset();
//        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
//            // Display it for the driver.
//            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
//            telemetry.update();
//        }
//    }
//
//
//
//    // ENCODER FUNCTIONS
//    public void resetEncoder()
//    {
//        robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//    public void startEncoderMode()
//    {
//        //Set Encoder Mode
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//
//
//    //        //RAISE ARM FUNCTION
//    public void raise(double count) {
//
//        int newArmLiftTarget;
//
//        // Determine new target position, and pass to motor controller
//        newArmLiftTarget = robot.armLift.getCurrentPosition() + (int) (count);
//        robot.armLift.setTargetPosition(newArmLiftTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.armLift.setPower(Math.abs(robot.DRIVE_SPEED));
//
//    }
//
//
//
////        //LOWER ARM FUNCTION
////        public void lower(double count) {
////
////            int newElbowMotorTarget;
////
////            // Determine new target position, and pass to motor controller
////            newElbowMotorTarget = robot.elbowMotor.getCurrentPosition() - (int) (count);
////            robot.elbowMotor.setTargetPosition(newElbowMotorTarget);
////
////            // Turn On RUN_TO_POSITION
////            robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////            robot.elbowMotor.setPower(Math.abs(robot.DRIVE_SPEED));
////
////        }
////
////
////
////        public void placeFreight(){
////            //move motor down
////            robot.elbowMotor.setPower(0.2);
////            runtime.reset();
////            while (runtime.seconds() < 0.6){
////            }
////
////            //unclamp servo
////            robot.elbowMotor.setPower(0);
////            robot.freightSnatcher1.setPosition(0.6);
////
////            //wait
////            runtime.reset();
////            while (runtime.seconds() < 1){
////            }
////
////            //move arm back up
////            robot.elbowMotor.setPower(-0.2);
////            runtime.reset();
////            while (runtime.seconds() < 0.5){
////            }
////
////            robot.elbowMotor.setPower(0);
////        }
//}