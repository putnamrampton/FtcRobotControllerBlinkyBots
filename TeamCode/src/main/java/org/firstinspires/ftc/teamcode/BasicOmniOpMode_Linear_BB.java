/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
//@Disabled
public class BasicOmniOpMode_Linear_BB extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //telescoping arm
    private DcMotor armDrive = null;

    //servo claw
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   clawServo;
    // Servo   hingeServo; // Standard servo
    CRServo   hingeServo; // Continuous servo
    double  clawPosition = 0.50;  //guessing middle is 0.50
    // double  hingePosition = 0.10; // Standard servo
    double hingePower = 0; // Continuous servo.  0.5 is off.  0.0 and 1.0 are the min and max.
    //boolean rampUp = true;

/*
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 */

    /*
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    */

        @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        //add config for new motor (telescoping arm)
        armDrive = hardwareMap.get(DcMotor.class, "arm_drive");
        armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        clawServo = hardwareMap.get(Servo.class, "claw_hand");

        // Connect to servo
        hingeServo = hardwareMap.get(CRServo.class, "arm_hinge");

        // Send telemetry message to indicate successful Encoder reset
        //telemetry.addData("Arm location starting at",  "%7d",
                //armDrive.getCurrentPosition());

            // Wait for the start button
        // telemetry.addData(">", "Press Start to scan Servo." );
        // telemetry.update();
        waitForStart();




            // ########################################################################################
            // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
            // ########################################################################################
            // Most robots need the motors on one side to be reversed to drive forward.
            // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
            // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
            // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
            // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
            // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
            // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            // Set direction for the telescoping arm
            armDrive.setDirection(DcMotor.Direction.REVERSE);

            // Wait for the game to start (driver presses START)
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            //int targetTicks = 0;

            //start with the claw closed
            //boolean clawOpen= false;

        // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double max;
                //double max2;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower = axial - lateral + yaw;
                double rightBackPower = axial + lateral - yaw;

                //gamepad2 - control the arms - right joystick up to extend, down to retract
                double axial2 = -gamepad2.right_stick_y;  // Note: pushing stick forward gives negative value

                //calculate arm power based on joystick from gamepad2
                double armPower = axial2 * 0.65;
                //Overiding armPower to keep the telescoping arm from falling
                if (armPower == 0) {
                    armPower = 0.05;
                }



                //gamepad2 - control arm hinge - dpad_down pressed for lowering
                boolean dpadDownPressed = gamepad2.dpad_down;  // dpad down button gamepad 2

                //gamepad2 - control claw intake - dpad_up pressed for raising
                boolean dpadUpPressed = gamepad2.dpad_up;  // dpad up button gamepad 2

                // Control hinge movement through buttons
                // TODO: find the right values by testing
                if (dpadDownPressed) { //hinge down
                    hingePower = -1.0;
                } else if (dpadUpPressed) { //hinge up
                    hingePower = 1.0;
                } else {
                    hingePower = 0.04;
                }

                /*
                // Control hinge movement through buttons
                // TODO: find the right values by testing
                if (dpadDownPressed) { //hinge down
                    hingePosition = 0.40;
                } else if (dpadUpPressed) { //hinge up
                    hingePosition = 0.10;
                }
                */

                //gamepad2 - control claw intake - a pressed for open
                boolean buttonAPressed = gamepad2.a;  // A gamepad 2

                //gamepad2 - control claw intake - b pressed for close
                boolean buttonBPressed = gamepad2.b;  // B gamepad 2

                // Control claw movement through buttons
                // TODO: find the right values by testing
                if (buttonAPressed) { //open the claw
                    clawPosition = 0.50;
                } else if (buttonBPressed) { //close the claw
                    clawPosition = 0.90;
                }

                /*
                Encoder mode
                if (gamepad2.dpad_down) {
                targetTicks = 0;
                 } else if (gamepad2.dpad_up) {
                     targetTicks = 3700; //Depends where our goal position is for the arm - this will intel our final target tick amount
                }
                */



                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                int divider = 3;
                // if button LB is pressed, the speed will increase
                boolean buttonLBPressed = gamepad1.left_bumper;  // B gamepad 1
                if (buttonLBPressed == false) {
                    leftFrontPower /= divider;
                    rightFrontPower /= divider;
                    leftBackPower /= divider;
                    rightBackPower /= divider;
                }

                // This is test code:
                //
                // Uncomment the following code to test your motor directions.
                // Each button should make the corresponding motor run FORWARD.
                //   1) First get all the motors to take to correct positions on the robot
                //      by adjusting your Robot Configuration if necessary.
                //   2) Then make sure they run in the correct direction by modifying the
                //      the setDirection() calls above.
                // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

                // Send calculated power to wheels
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);

                // Send calculated power to telescoping arm
                armDrive.setPower(armPower);

                //encoderDrive(0.5, targetTicks, 2);

                // Set the servo to the new position and pause;
                clawServo.setPosition(clawPosition);

                //hingeServo.setPosition(hingePosition); // Standard servo

                hingeServo.setPower(hingePower); // Continuous servo

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.addData("Arm", "%5.2f", armPower);
                // telemetry.update();

                // Display the current value
                // telemetry.addData("Servo Max", "%5.2f", MAX_POS);
                // telemetry.addData("Servo Min", "%5.2f", MIN_POS);
                telemetry.addData("Claw Position", "%5.2f", clawPosition);
                //telemetry.addData("Hinge Position", "%5.2f", hingePosition); // Standard servo
                telemetry.addData("Hinge Power", "%4.2f", hingePower);
                telemetry.addData(">", "Press Stop to end test.");
                telemetry.update();

                sleep(CYCLE_MS);
                idle();
            }

            // Signal done;
            telemetry.addData(">", "Done");
            telemetry.update();
        }

    /*
     *  Method to perform a move to an exact position, based on encoder counts.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    /*public void encoderDrive(double speed,
                             int targetTicks,
                             double timeoutS) {
        //if your already at the target do nothing
        if (armDrive.getCurrentPosition() == targetTicks) {
            return;
        }
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            armDrive.setTargetPosition(targetTicks);

            // Turn On RUN_TO_POSITION
            armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            armDrive.setPower(Math.abs(speed));

            //TODO: figure out how to include the while (opModeISActive() loop in code above
            //the encoder resolution is 537.7 PPR, this is for a 19.2:1 gear ratio
            //initial guess for fully extending the slide: about 15,000 - WAY TO MUCH

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    armDrive.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Running telescoping arm to",  " %7d", targetTicks);
                telemetry.addData("Currently at",  " at %7d",
                        armDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            armDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }

        */
    }


