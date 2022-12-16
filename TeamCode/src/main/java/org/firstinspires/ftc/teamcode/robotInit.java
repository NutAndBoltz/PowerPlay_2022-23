package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


//@Disabled
public class robotInit {

    /* Public OpMode members. */
    //creating objects
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;
    public DcMotor armLiftLeft; //arm lifting mechanism
    public DcMotor armLiftRight; //arm lifting mechanism
    public DcMotor waiter; //the thing that spins the arm like a turntable


//    public CRServo spinner; // spins the motor to bring in the cone
    public Servo closerL; // clamp and release cone
    public Servo closerR; // clamp and release cone

    //from Encoder Sample
    double     COUNTS_PER_MOTOR_REV    = 537.7 ;
    double     WHEEL_DIAMETER_INCHES   = 4.0 ;  // For figuring circumference
    double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    double     DRIVE_SPEED             = 0.35;
    double     teleOP_FORWARD_SPEED    = 1;

    /* local OpMode members. */
    HardwareMap hardwareMap ;

    // instantiate = to create an instance of = constructor
    /* Constructor */
    public robotInit(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;


        // Define and Initialize Motors
        motorFL = hardwareMap.get(DcMotor.class, "motor_fl");
        motorFR = hardwareMap.get(DcMotor.class, "motor_fr");
        motorBL = hardwareMap.get(DcMotor.class, "motor_bl");
        motorBR = hardwareMap.get(DcMotor.class, "motor_br");
        armLiftLeft = hardwareMap.get(DcMotor.class, "armLiftLeft");
        armLiftRight = hardwareMap.get(DcMotor.class, "armLiftRight");
        waiter = hardwareMap.get(DcMotor.class, "waiter");

        // Set the direction of the DC motors
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        armLiftLeft.setDirection(DcMotor.Direction.REVERSE);
        armLiftRight.setDirection(DcMotor.Direction.FORWARD);
        waiter.setDirection(DcMotor.Direction.REVERSE);

        // Set all DC motors to zero power
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorFL.setPower(0);
        armLiftLeft.setPower(0);
        armLiftRight.setPower(0);
        waiter.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLiftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLiftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waiter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize ALL installed servos.
//        spinner = hardwareMap.get(CRServo.class, "spinner");
        closerL = hardwareMap.get(Servo.class, "closerL");
        closerR = hardwareMap.get(Servo.class, "closerR");

        //init servos
        //freightSnatcher1.setPosition(0.72);
        //freightSnatcher2.setPosition(0.4);

    }
}