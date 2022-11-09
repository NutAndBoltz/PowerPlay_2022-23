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
//import org.openftc.easyopencv.OpenCvWebcam;
//
//@
//@Autonomous(name="Red Warehouse", group="Pushbot")
//public class RedWarehouse extends LinearOpMode {
//
//    public robotInit robot = new robotInit();
//    ElapsedTime runtime = new ElapsedTime();
//
//    int level; //for auto
//
//    //OpenCvCamera webcam;
//    DeterminationPipeline pipeline; //pipeline = series of img coming through camera to process
//
//    @Override
//    public void runOpMode() {
//
//        robot.init(hardwareMap);
//        telemetry.addData("Status:", "Robot init");
//        telemetry.update();
//        sleep(1000);
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        telemetry.addData("Status:", "waiting");
//        telemetry.update();
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
//        telemetry.addData("Status:", "Webcam made");
//        telemetry.update();
//        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        telemetry.addData("Status:", "Delivered");
//        telemetry.update();
//
//        pipeline = new DeterminationPipeline();
//        webcam.setPipeline(pipeline);
//
//        resetEncoder();
//        startEncoderMode();
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//
//        });
//
//
//        // Wait for the game to start (driver presses PLAY)
//        telemetry.addLine("Waiting for start");
//        telemetry.update();
//        waitForStart();
//
//        boolean opModeFinished = false;
//
//        while (opModeIsActive() && !opModeFinished) {
//
//
//            // Telemetry for testing barcode detection
//            telemetry.addData("Analysis1", pipeline.getAnalysis1());
//            telemetry.addData("Analysis2", pipeline.getAnalysis2());
//            telemetry.addData("Analysis3", pipeline.getAnalysis3());
//            telemetry.addData("Position", pipeline.position);
//            telemetry.update();
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(1000);
//
//
//            // STEP 1 - Detect element on barcode & store in level var
//            if (pipeline.position == webcamtest.DeterminationPipeline.ElementPosition.Level1) {
//                telemetry.addData("Detected", "level 1!");
//                telemetry.update();
//
//                level = 1;
//
//            } else if (pipeline.position == webcamtest.DeterminationPipeline.ElementPosition.Level2) {
//                telemetry.addData("Detected", "level 2!");
//                telemetry.update();
//
//                level = 2;
//
//            } else {
//                telemetry.addData("Detected", "level 3!");
//                telemetry.update();
//
//                level = 3;
//
//            }
//            sleep(1000);
//
//            //go toward hub
//            strafeLeft(23);
//            moveForward(17);
//
//            if (level == 1) {
//                telemetry.addData("Detected", "level 1!");
//                telemetry.update();
//
//                raise(-70);
//                sleep(100);
//                moveForward(3);
//
//            } else if (level == 2) {
//                telemetry.addData("Detected", "level 2!");
//                telemetry.update();
//
//                raise(-183);
//                sleep(100);
//                moveForward(2);
//
//            } else {
//                telemetry.addData("Detected", "level 3!");
//                raise(-200);
//                sleep(200);
//                moveForward(3);
//
//            }
//
//            // place freight on hub
//            robot.freightSnatcher1.setPower(-1); //vacuum spews out freight
//            runtime.reset();
//            while (runtime.seconds() < 1.5) {
//                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
//            robot.freightSnatcher1.setPower(0); //vacuum stops
//
//            //STEP 4 -- head to warehouse
//            moveBackward(18);
//            turnRight(18.5);
//            strafeRight(5);
//            moveForward(50);
//            strafeLeft(30);
//
//
//            opModeFinished = true;
//            //sleep(8000);
//        }
//
//    }
//
//
//
//    /* VUFORIA RING DETECTION */
//    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
//    {
//        /*
//         * An enum to define the skystone position
//         */
//        public enum RingPosition
//        {
//            FOUR,
//            ONE,
//            NONE
//        }
//
//        /*
//         * Some color constants
//         */
//        static final Scalar BLUE = new Scalar(0, 0, 255);
//        static final Scalar GREEN = new Scalar(0, 255, 0);
//
//        /*
//         * The core values which define the location and size of the sample regions
//         */
//        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(85,183);
//
//        static final int REGION_WIDTH = 50;
//        static final int REGION_HEIGHT = 50;
//
//        final int FOUR_RING_THRESHOLD = 135; //143-148
//        final int ONE_RING_THRESHOLD = 125; //119-131
//
//        Point region1_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        /*
//         * Working variables
//         */
//        Mat region1_Cb;
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//        int avg1;
//
//        // Volatile since accessed by OpMode thread w/o synchronization
//        public volatile RingPosition position = RingPosition.FOUR;
//
//        /*
//         * This function takes the RGB frame, converts to YCrCb,
//         * and extracts the Cb channel to the 'Cb' variable
//         */
//        void inputToCb(Mat input)
//        {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 1);
//        }
//
//        @Override
//        public void init(Mat firstFrame)
//        {
//            inputToCb(firstFrame);
//
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//        }
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            inputToCb(input);
//
//            avg1 = (int) Core.mean(region1_Cb).val[0];
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    BLUE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//            position = RingPosition.FOUR; // Record our analysis
//            if(avg1 > FOUR_RING_THRESHOLD){
//                position = RingPosition.FOUR;
//            }else if (avg1 > ONE_RING_THRESHOLD){
//                position = RingPosition.ONE;
//            }else{
//                position = RingPosition.NONE;
//            }
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    GREEN, // The color the rectangle is drawn in
//                    1); // Negative thickness means solid fill
//
//            return input;
//        }
//
//        public int getAnalysis()
//        {
//            return avg1;
//        }
//
//
//
//
//
//
//    // STEP 1 - Detect where the customized element is placed on the field
//    public static class DeterminationPipeline extends OpenCvPipeline {
//        public enum ElementPosition {
//            Box1,
//            Box2,
//            Box3
//        }
//
//        // Some color constants
//        static final Scalar TURQUOISE = new Scalar(163, 210, 202);
//
//        // The core values which define the location and size of the sample regions
//        static final Point BOX2_TOPLEFT_ANCHOR_POINT = new Point(123, 135);
//
//        static final int REGION_WIDTH = 75;
//        static final int REGION_HEIGHT = 75;
//
//
//        Point region1_pointA = new Point(
//                BOX2_TOPLEFT_ANCHOR_POINT.x,
//                BOX2_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                BOX2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                BOX2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//
//        // Creating variables
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//
//        //Box2
//        Mat region2_Cb;
//        int avg1;
//
//
//        // Volatile since accessed by OpMode thread w/o synchronization
//        public volatile webcamtest.DeterminationPipeline.ElementPosition position = webcamtest.DeterminationPipeline.ElementPosition.Level1;
//
//        // This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
//        void inputToCb(Mat input) {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 1);
//        }
//
//        @Override
//        public void init(Mat firstFrame) {
//            inputToCb(firstFrame);
//
//            //Figure out how much blue is in each region
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//        }
//
//        @Override
//        public Mat processFrame(Mat input) {
//            inputToCb(input);
//
//            avg1 = (int) Core.mean(region1_Cb).val[0];
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    TURQUOISE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//
//            position = webcamtest.DeterminationPipeline.ElementPosition.Level1; // Record our analysis
//            //find the box/region with maximum red color
//            if (avg1) {
//                position = webcamtest.DeterminationPipeline.ElementPosition.Level1;
//            }
//
//            return input;
//        }
//
//        //TODO check to see if this works :)
//        public int getAnalysis2() {
//            return avg1;
//        }
//    }
//
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
//        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
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
//        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("Strafing right", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
//            telemetry.update();
//        }
//    }
//
//
//
//
//    // FUNCTION TO TURN Right
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
//        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
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
//        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
//            telemetry.update();
//        }
//    }
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
//        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
//            telemetry.update();
//        }
//    }
//
//
//
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
//        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
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
//        robot.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//    public void startEncoderMode()
//    {
//        //Set Encoder Mode
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//
//
//    //RAISE ARM FUNCTION
//    public void raise(double count) {
//
//        int Target1;
//        int Target2;
//
//        // Determine new target position, and pass to motor controller
//        Target1 = robot.elbowMotor.getCurrentPosition() + (int) (count);
//        Target2 = robot.armLift.getCurrentPosition() + (int) (count);
//        robot.elbowMotor.setTargetPosition(Target1);
//        robot.armLift.setTargetPosition(Target2);
//
//        // Turn On RUN_TO_POSITION
//        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.elbowMotor.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.armLift.setPower(Math.abs(robot.DRIVE_SPEED));
//
//    }
//
//
//
//    //LOWER ARM FUNCTION
//    public void lower(double count) {
//
//        int newElbowMotorTarget;
//
//        // Determine new target position, and pass to motor controller
//        newElbowMotorTarget = robot.elbowMotor.getCurrentPosition() - (int) (count);
//        robot.elbowMotor.setTargetPosition(newElbowMotorTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.elbowMotor.setPower(Math.abs(robot.DRIVE_SPEED));
//
//    }
//}
