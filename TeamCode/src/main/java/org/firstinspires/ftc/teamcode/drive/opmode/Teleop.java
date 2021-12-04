/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;



/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Hunga Munga: Teleop", group="Iterative Opmode")

public class Teleop extends OpMode

{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx intakeDrive = null;
    private DcMotorEx carouselMotor = null;
    private Servo outtakeServo1 = null;
    private Servo outtakeServo2 = null;
    private DcMotorEx tunnelDrive =  null;
    private DcMotorEx elevatorDrive = null;

//    DcMotor tester = null;
//    DcMotorEx.RunMode.RUN_TO_POSITION;

//carousel and outake reverse

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");
        intakeDrive = hardwareMap.get(DcMotorEx.class, "intake");
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carousel");
        outtakeServo1 = hardwareMap.get(Servo.class, "outtake1");
        outtakeServo2 = hardwareMap.get(Servo.class, "outtake2");
        //tunnelDrive = hardwareMap.get(DcMotorEx.class, "tunnel");
        elevatorDrive = hardwareMap.get(DcMotorEx.class, "elevator");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        intakeDrive.setDirection(DcMotorEx.Direction.FORWARD);
        carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);
        outtakeServo1.setDirection(Servo.Direction.FORWARD);
        outtakeServo2.setDirection(Servo.Direction.REVERSE);
        //tunnelDrive.setDirection(DcMotorEx.Direction.FORWARD);
        elevatorDrive.setDirection(DcMotorEx.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        //Setting Zero Power behaviour
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //tunnelDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);






        //setting PID coefficients
        leftFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        rightFrontDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        leftBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        rightBackDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        intakeDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
        carouselMotor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        //tunnelDrive.setVelocityPIDFCoefficients(15, 0, 0, 0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double strafe = gamepad1.left_stick_x;
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double intakeIn = gamepad1.left_trigger;
        double intakeOut = gamepad1.right_trigger;
//        double intakeIn = gamepad1.right_trigger;
//        double intakeOut = gamepad1.left_trigger;
        boolean carouselCounterClock = gamepad1.a;
        boolean carouselClockWise = gamepad1.y;
        boolean outtakeIn = gamepad1.left_bumper;
        boolean outtakeOut = gamepad1.right_bumper;
        boolean elevatorUp = gamepad1.dpad_up;
        boolean elevatorDown = gamepad1.dpad_down;

//        Correct equations:
        leftFrontPower   = drive + strafe + turn;
        rightFrontPower  = drive - strafe - turn;
        leftBackPower    = drive - strafe + turn;
        rightBackPower   = drive + strafe - turn;

//        leftFrontPower   = -drive - strafe - turn;
//        rightFrontPower  = -drive - strafe + turn;
//        leftBackPower    = -drive + strafe - turn;
//        rightBackPower   = -drive + strafe + turn;

        double maxValue = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

        if (maxValue > 1) {
            leftFrontPower /= maxValue;
            rightFrontPower /= maxValue;
            leftBackPower /= maxValue;
            rightBackPower /= maxValue;
        }

        if (intakeIn > 0) {
            //intakeDrive.setVelocity(intakeIn * 1000);
            intakeDrive.setPower(intakeIn);
        }
        else if (intakeOut > 0){
            //intakeDrive.setVelocity(intakeOut * -1000);
            intakeDrive.setPower(-intakeOut);
        }
        else {
            intakeDrive.setPower(0);
        }




        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated velocity to wheels
        leftFrontDrive.setVelocity(leftFrontPower * 3000);
        rightBackDrive.setVelocity(rightBackPower * 3000);
        leftBackDrive.setVelocity(leftBackPower * 3000);
        rightFrontDrive.setVelocity(rightFrontPower * 3000);

        // intake objects in and out when the corresponding trigger is pressed
        if (intakeIn > 0) {
            intakeDrive.setVelocity(intakeIn * -4000);
            //tunnelDrive.setVelocity(intakeIn * -4000);
            //intakeDrive.setPower(intakeIn);
        }
        else if (intakeOut > 0){
            intakeDrive.setVelocity(intakeOut * 4000);
            //intakeDrive.setPower(-intakeOut);
        }
        else {
            intakeDrive.setPower(0);
        }


        //carousel clockwise and counter clockwise spin when the corresponding button is pressed
        if (carouselCounterClock == true) {
            carouselMotor.setVelocity(2800 * 1);
        }
        else if (carouselClockWise == true) {
            carouselMotor.setVelocity(2800 * -1);
        }
        else {
            carouselMotor.setVelocity(0);
        }

        //outtake in and out when the corresponding button is pressed.
        if (outtakeOut== true) {
            outtakeServo1.setPosition(1);
            outtakeServo2.setPosition(1);
        }
        else if (outtakeIn == true) {
            outtakeServo1.setPosition(0);
            outtakeServo2.setPosition(0);
        }
        else {
            outtakeServo1.setPosition(.5);
            outtakeServo2.setPosition(.5);
        }

        if (elevatorUp == true) {
            elevatorDrive.setVelocity(1200);
        }
        else if (elevatorDown == true) {
            elevatorDrive.setVelocity(-1200);
        }
        else{
            elevatorDrive.setVelocity(0);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left front (%.2f)", leftFrontDrive.getVelocity());
        telemetry.addData("Motors", "right front (%.2f)", rightFrontDrive.getVelocity());
        telemetry.addData("Motors", "left back (%.2f)", leftBackDrive.getVelocity());
        telemetry.addData("Motors", "right back (%.2f)", rightBackDrive.getVelocity());
        telemetry.addData("Motors", "intake speed (%.2f)", intakeDrive.getVelocity());
        telemetry.addData("Boolean", "intake in(%b)", intakeIn);
        telemetry.addData("Boolean", "intake out(%b)", intakeOut);
        telemetry.addData("Boolean", "carousel(%b)", carouselClockWise);
        telemetry.addData("boolean", "carousel(%b", carouselCounterClock);
        telemetry.addData("boolean", "outtake(%b)", outtakeIn);
        telemetry.addData("Boolean", "outtake(%b)", outtakeOut);
        telemetry.addData("Motors", "elevator(%.2f)", elevatorDrive.getVelocity());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}