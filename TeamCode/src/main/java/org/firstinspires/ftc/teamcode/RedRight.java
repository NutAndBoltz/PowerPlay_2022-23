package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
//ignore this
@TeleOp
public class RedRight extends LinearOpMode {
    public robotInit robot = new robotInit();
    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        boolean openToggle = false;

//        robot.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            double vertical = 0.55*(gamepad1.left_stick_y); //move forward, backward
            double horizontal = 0.55*(gamepad1.left_stick_x); //move left, right
            double turn = 0.55*(-gamepad1.right_stick_x); //turn left, right
            double armUp = -gamepad1.right_trigger; // brings linear slides up
            double armDown = gamepad1.left_trigger; // brings linear slides down
            boolean clamp = gamepad1.dpad_left; // clamps the closer servo
            boolean release = gamepad1.dpad_right; // release the closer servo


            //driving and arm control
            robot.motorFL.setPower(vertical + horizontal + turn);
            robot.motorFR.setPower(vertical + horizontal - turn);
            robot.motorBL.setPower(vertical - horizontal - turn);
            robot.motorBR.setPower(vertical - horizontal + turn);
            robot.armLift.setPower(armDown);
            robot.armLift.setPower(armUp);

            //clamp and release cone with closer servo
            if (clamp) {
                robot.closer.setPosition(0.8); //clamp cone with closer servo
                telemetry.addData("Path1",  "Clamp cone");
                telemetry.update();
            }
            if (release) {
                robot.closer.setPosition(0.2); //release cone with closer servo
                telemetry.addData("Path1",  "Release cone");
                telemetry.update();
            }

            //rotates counter-clockwise (spews out the cone)
            if (gamepad1.left_bumper) {
                robot.spinner.setPower(0.5);
                telemetry.addData("Path1",  "Release cone");
                telemetry.update();
            }
            //rotates clockwise (brings in the cone)
            if (gamepad1.right_bumper) {
                robot.spinner.setPower(0.15);
                telemetry.addData("Path1",  "Clamp cone");
                telemetry.update();
            }
        }
    }
}



    /* FUNCTIONS */

//    public void raise(double count) {
//
//        int newElbowMotorTarget;
//
//        // Determine new target position, and pass to motor controller
//        newElbowMotorTarget = robot.elbowMotor.getCurrentPosition() + (int)(count);
//        robot.elbowMotor.setTargetPosition(newElbowMotorTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.elbowMotor.setPower(0.3);
//
//        runtime.reset();
//        while (opModeIsActive() && robot.elbowMotor.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d", newElbowMotorTarget);
//            telemetry.update();
//        }
//    }
//
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
//        robot.elbowMotor.setPower(0.3);
//
//        runtime.reset();
//        while (opModeIsActive() && robot.elbowMotor.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d", newElbowMotorTarget);
//            telemetry.update();
//        }