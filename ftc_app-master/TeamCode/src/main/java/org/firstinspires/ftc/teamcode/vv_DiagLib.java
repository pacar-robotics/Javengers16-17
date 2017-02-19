package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.w3c.dom.NodeList;

import static org.firstinspires.ftc.teamcode.vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.vv_Constants.ARM_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.ARM_MOTOR_ENCODER_MARGIN;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.CAP_BALL_ENCODER_MARGIN;
import static org.firstinspires.ftc.teamcode.vv_Constants.CAP_BALL_POSITION_INCREMENT;
import static org.firstinspires.ftc.teamcode.vv_Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.INTAKE_INCREMENT;
import static org.firstinspires.ftc.teamcode.vv_Constants.INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_POSITION_INCREMENT;
import static org.firstinspires.ftc.teamcode.vv_Constants.LAUNCH_POWER_INCREMENT;
import static org.firstinspires.ftc.teamcode.vv_Constants.MAX_ROBOT_DIAGNOSTIC_TESTS;
import static org.firstinspires.ftc.teamcode.vv_Constants.MECCANUM_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.vv_Constants.MECCANUM_WHEEL_ENCODER_MARGIN;
import static org.firstinspires.ftc.teamcode.vv_Constants.ROBOT_TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.vv_Constants.TURN_POWER;
import static org.firstinspires.ftc.teamcode.vv_Constants.WORM_DRIVE_ENCODER_MARGIN;

/**
 * Created by thomas on 9/25/2016.
 */

public class vv_DiagLib {

    protected vv_Robot robot;
    RobotTest robotTestArray[];

    public vv_DiagLib(vv_OpMode aOpMode)
            throws InterruptedException {
        robot = new vv_Robot();
        robot.init(aOpMode, aOpMode.hardwareMap);

        //initialize the array of tests

        //first lets create the space for the tests
        robotTestArray = new RobotTest[MAX_ROBOT_DIAGNOSTIC_TESTS];

        //now lets actually initialize the array with class instances.

        for (int i = 0; i < robotTestArray.length; i++) {
            robotTestArray[i] = new RobotTest();
        }
        //lets push the tests we have into the array.


        invalidateAllTests(aOpMode);

        createTestEntries(aOpMode);


        aOpMode.DBG("Before Invalidate all Results");

        invalidateAllTestResults(aOpMode);
    }

    public RobotTest findTestByName(vv_OpMode aOpMode, String name) {
        //could do this in a hashMap but since the number of records is so small we can use a simple loop.
        // the cost of a string compare is ok for our small number of records.

        for (int i = 0; i < robotTestArray.length; i++) {
            if (robotTestArray[i].getTestValidity(aOpMode)) {
                //the test is valid
                if (robotTestArray[i].getTestName(aOpMode).equals(name)) {
                    //found test
                    return (robotTestArray[i]);
                }
            }

        }
        //we have not found the record, return null.
        return null;

    }

    public void runAllTests(vv_OpMode aOpMode) throws InterruptedException {
        for (int i = 0; i < robotTestArray.length; i++) {
            if (robotTestArray[i].getTestValidity(aOpMode)) {
                //runnable test, it has been initialized
                //lets run and store the test.
                aOpMode.telemetryAddData("Running Test:", "Named:",
                        robotTestArray[i].getTestName(aOpMode) +
                                robotTestArray[i].getTestShortDescription(aOpMode));
                aOpMode.telemetryUpdate();
                robotTestArray[i].getTestRunnableTest(aOpMode).
                        runTest(aOpMode, robotTestArray[i]); //we expect the runTest itself will
                //set all the values of the record properly.
            }
        }

    }

    public void runAllAutomaticTests(vv_OpMode aOpMode) throws InterruptedException {
        int validTestCount = getValidTestCount(aOpMode);
        for (int i = 0; i < robotTestArray.length; i++) {
            if ((robotTestArray[i].getTestValidity(aOpMode)) &&
                    (robotTestArray[i].getTestType(aOpMode) == TestType.AUTOMATIC)) {
                //runnable test, it has been initialized and it is an automatic test

                //lets run and store the test.
                aOpMode.telemetryAddData("Running:", "Test Number:" + i+1 + "/" + validTestCount + ":",
                        robotTestArray[i].getTestName(aOpMode) +
                                robotTestArray[i].getTestShortDescription(aOpMode));
                aOpMode.telemetryUpdate();
                robotTestArray[i].getTestRunnableTest(aOpMode).
                        runTest(aOpMode, robotTestArray[i]); //we expect the runTest itself will
                //set all the values of the record properly.
            }
        }

    }

    public int getValidTestCount(vv_OpMode aOpMode) {
        int validTestCount = 0;
        for (int i = 0; i < robotTestArray.length; i++) {
            if (robotTestArray[i].getTestValidity(aOpMode)) {
                validTestCount++;
            }
        }
        return validTestCount;
    }

    public void initializeAllTests(vv_OpMode aOpMode) {
        for (int i = 0; i < robotTestArray.length; i++) {
            robotTestArray[i].setTestValidity(aOpMode, false); //invalidate each test
            robotTestArray[i].setTestResultValidity(aOpMode, false); //invalidate each test result
        }
    }

    public void invalidateAllTests(vv_OpMode aOpMode) {
        for (int i = 0; i < robotTestArray.length; i++) {
            robotTestArray[i].setTestValidity(aOpMode, false); //invalidate each test
        }
    }


    //These are the tests

    public void invalidateAllTestResults(vv_OpMode aOpMode) {
        for (int i = 0; i < robotTestArray.length; i++) {
            robotTestArray[i].setTestResultValidity(aOpMode, false); //invalidate each test result
        }
    }

    private void createTestEntries(vv_OpMode aOpMode) {
        robotTestArray[0].createTest(aOpMode, "testFrontLeftMotor", 0, "Test Front Left Motor",
                "Tests the front left motor by running it for a small duration",
                TestType.AUTOMATIC, new TestFrontLeftWheel());

        robotTestArray[1].createTest(aOpMode, "testFrontRightMotor", 1, "Test Front Right Motor",
                "Tests the front right motor by running it for a small duration",
                TestType.AUTOMATIC, new TestFrontRightWheel());

        robotTestArray[2].createTest(aOpMode, "testBackLeftMotor", 2, "Test Back Left Motor",
                "Tests the back left motor by running it for a small duration",
                TestType.AUTOMATIC, new TestBackLeftWheel());

        robotTestArray[3].createTest(aOpMode, "testBackRightMotor", 3, "Test Back Right Motor",
                "Tests the back right motor by running it for a small duration",
                TestType.AUTOMATIC, new TestBackRightWheel());

        robotTestArray[4].createTest(aOpMode, "testPlatformLeft", 4, "Test Platform Left",
                "Tests the Platform by moving it left for a small duration",
                TestType.AUTOMATIC, new TestPlatformLeft());

        robotTestArray[5].createTest(aOpMode, "testPlatformRight", 5, "Test Platform Right",
                "Tests the Platform by moving it right for a small duration",
                TestType.AUTOMATIC, new TestPlatformRight());

        robotTestArray[6].createTest(aOpMode, "testPlatformForward", 6, "Test Platform Forward",
                "Tests the Platform by moving it forward for a small duration",
                TestType.AUTOMATIC, new TestPlatformForward());

        robotTestArray[7].createTest(aOpMode, "testPlatformBackward", 7, "Test Platform Backward",
                "Tests the Platform by moving it backward for a small duration",
                TestType.AUTOMATIC, new TestPlatformBackward());

        robotTestArray[8].createTest(aOpMode, "testPlatformDiagonals", 8, "Test Platform Diagonal Movements",
                "Tests the Platform Diagonal Movement by moving it for a small distance",
                TestType.AUTOMATIC, new TestPlatformDiagonals());

        robotTestArray[9].createTest(aOpMode, "testPlatformRotation", 9, "Test Platform Rotation",
                "Tests the Platform by rotating it for a fixed angle",
                TestType.AUTOMATIC, new TestPlatformRotation());

        robotTestArray[10].createTest(aOpMode, "testRangeSensor", 10, "Test Ultrasonic Range Sensor",
                "Tests the Range sensing by rotating the platform and checking for change in Range",
                TestType.AUTOMATIC, new TestRangeSensor());

        robotTestArray[11].createTest(aOpMode, "testFloorSensor", 11, "Test Floor Color Sensor",
                "Tests the Floor Color sensing by rotating the platform and checking for change in Color Value",
                TestType.AUTOMATIC, new TestFloorSensor());

        robotTestArray[12].createTest(aOpMode, "testIntake", 12, "Test Intake",
                "Tests the Intake by checking for change in Encoder Value",
                TestType.AUTOMATIC, new TestIntake());

        robotTestArray[13].createTest(aOpMode, "testChooChooLaunch", 13, "Test Choo Choo Launch Arm",
                "Tests the Choo Choo Launch by checking for change in Encoder Value",
                TestType.AUTOMATIC, new TestChooChooLaunch());

        robotTestArray[14].createTest(aOpMode, "testCapBall", 14, "Test Cap Ball Lift",
                "Tests the Cap Ball Lift by Moving Lift and checking for change in Encoder Value",
                TestType.AUTOMATIC, new TestCapBall());

        robotTestArray[15].createTest(aOpMode, "testChooChooTension", 15, "Test Choo Choo Tensioning",
                "Tests the Choo Choo Tensioning Mechanism by checking for change in Encoder Value",
                TestType.AUTOMATIC, new TestChooChooTension());







    }


    //=================end test entries


    public void analyzeTestResults(vv_OpMode aOpMode) throws InterruptedException {
        boolean noErrorsFound = true;
        //lower the flag in prep for test
        robot.setBallFlagServoPosition(aOpMode,
                vv_Constants.BALL_FLAG_SERVO_LOWERED);

        for (int i = 0; i < robotTestArray.length; i++) {
            //list all tests and results.
            if (robotTestArray[i].getTestValidity(aOpMode)) {
                //its a valid test
                if (robotTestArray[i].getTestResultValidity(aOpMode)) {

                    //its a valid result
                    if (robotTestArray[i].getTestResult(aOpMode)) {
                        //this test passed.
                        //lets print this message out.
                        aOpMode.telemetryAddData("Test Result:",
                                robotTestArray[i].getTestName(aOpMode),
                                ":" + "Test Passed");
                        aOpMode.telemetryUpdate();
                        Thread.sleep(500); //wait on display

                    } else {
                        //this test failed.
                        //lets print out the message.
                        //flag the error in the boolean tracker
                        noErrorsFound = false;
                        //raise the flag.
                        robot.setBallFlagServoPosition(aOpMode,
                                vv_Constants.BALL_FLAG_SERVO_ALARM);

                        aOpMode.telemetryAddData("Test Result:",
                                robotTestArray[i].getTestName(aOpMode),
                                ":" + "Test Failed");
                        aOpMode.telemetryAddData("Test Result Message:", "",
                                robotTestArray[i].getTestResultMessage(aOpMode));

                        switch (robotTestArray[i].getTestSeverity(aOpMode)) {
                            case CRITICAL:
                                aOpMode.telemetryAddData("Test Result:",
                                        "This is a CRITICAL SEVERITY ERROR",
                                        "Error");
                                break;

                            case HIGH:
                                aOpMode.telemetryAddData("Test Result:",
                                        "This is a HIGH SEVERITY",
                                        "Error");
                                break;
                            case MEDIUM:
                                aOpMode.telemetryAddData("Test Result:",
                                        "This is a MEDIUM SEVERITY ERROR",
                                        "Error");
                                break;
                            case LOW:
                                break;
                            case INFO:
                                break;
                        }

                        aOpMode.telemetryAddData("Recommendation:", "try this:",
                                robotTestArray[i].getTestRecommendation(aOpMode));
                        aOpMode.telemetryUpdate();
                        //wait and display our errors if any
                        Thread.sleep(500); //wait on display

                    }

                }


            }

        }
    }

    public void writeAllResults(vv_OpMode aOpMode) throws InterruptedException {
        vv_XmlLib vvXmlLib = new vv_XmlLib(aOpMode);

        //initialize the XML tree for writing Diagnostic Results.
        vvXmlLib.initDiagResultsXmlForWrite(aOpMode);

        for (int i = 0; i < robotTestArray.length; i++) {
            //list all tests and results.
            if (robotTestArray[i].getTestValidity(aOpMode)) {
                //its a valid test
                if (robotTestArray[i].getTestResultValidity(aOpMode)) {
                    //its a valid result
                    //lets add it to the XML file
                    vvXmlLib.addRobotTestResultXML(aOpMode, robotTestArray[i]);
                }

            }
        }
        //We have completed writing of all the tags in the XML DOM that have valid results.
        //lets write out the XML file.
        vvXmlLib.writeDiagResultsXML(aOpMode);
    }

    public void readAllResults(vv_OpMode aOpMode) throws InterruptedException {
        vv_XmlLib vvXmlLib = new vv_XmlLib(aOpMode);

        //initialize the XML tree for reading Diagnostic Results.
        //this opens the DiagResults xml file and reads in the DOM.
        vvXmlLib.initDiagResultsXmlForRead(aOpMode);

        //read the Robot Test NodesList
        NodeList robotTestNodes = vvXmlLib.getRobotTestNodesFromDom(aOpMode);

        //now step through the list of RobotTestNodes and load the robotTestArray
        //but first initialize all tests to have clean data

        initializeAllTests(aOpMode);

        for (int i = 0; i < robotTestNodes.getLength() && i < robotTestArray.length; i++) {
            //lets step through each node.
            //first get a list of all child nodes of the test
            NodeList detailsNodeList = robotTestNodes.item(i).getChildNodes();
            for (int j = 0; i < detailsNodeList.getLength(); j++) {
                //depending on the type of node we need to assign it to the right Array
                switch (detailsNodeList.item(j).getNodeName()) {
                    case "testId":
                        robotTestArray[i].
                                setTestElementId(aOpMode, Integer.valueOf(detailsNodeList.item(j).getTextContent()));
                        break;

                    case "testName":
                        robotTestArray[i].
                                setTestElementName(aOpMode, detailsNodeList.item(j).getTextContent());
                        break;
                    case "shortDescription":
                        robotTestArray[i].
                                setTestShortDescription(aOpMode, detailsNodeList.item(j).getTextContent());
                        break;
                    case "longDescription":
                        robotTestArray[i].
                                setTestLongDescription(aOpMode, detailsNodeList.item(j).getTextContent());
                        break;
                    case "testResult":
                        if (detailsNodeList.item(j).getTextContent().equals("Passed")) {
                            robotTestArray[i].
                                    setTestResult(aOpMode, true);
                        } else {
                            robotTestArray[i].
                                    setTestResult(aOpMode, false);
                        }
                        break;
                    case "testResultMessage":
                        robotTestArray[i].
                                setTestResultMessage(aOpMode, detailsNodeList.item(j).getTextContent());
                        break;
                    case "testResultSeverity":
                        switch (detailsNodeList.item(j).getTextContent()) {
                            case "CRITICAL":
                                robotTestArray[i].
                                        setTestResultSeverity(aOpMode, ResultSeverity.CRITICAL);
                                break;

                            case "HIGH":
                                robotTestArray[i].
                                        setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                                break;
                            case "MEDIUM":
                                robotTestArray[i].
                                        setTestResultSeverity(aOpMode, ResultSeverity.MEDIUM);
                                break;
                            case "LOW":
                                robotTestArray[i].
                                        setTestResultSeverity(aOpMode, ResultSeverity.LOW);
                                break;
                            case "INFO":
                                robotTestArray[i].
                                        setTestResultSeverity(aOpMode, ResultSeverity.INFO);
                                break;
                            case "UNKNOWN":
                                robotTestArray[i].
                                        setTestResultSeverity(aOpMode, ResultSeverity.UNKNOWN);
                                break;
                            default:
                                robotTestArray[i].
                                        setTestResultSeverity(aOpMode, ResultSeverity.UNKNOWN);
                                break;
                        }

                    case "testRecommendation":
                        robotTestArray[i].
                                setTestRecommendation(aOpMode, detailsNodeList.item(j).getTextContent());
                    default:
                        aOpMode.telemetryAddData("Unknown text in node name:",
                                "Value:", detailsNodeList.item(j).getNodeName());
                        aOpMode.telemetryUpdate();
                        Thread.sleep(2000);
                }
            }
        }

        
        //We have completed writing of all the tags in the XML DOM that have valid results.
        //lets write out the XML file.
        vvXmlLib.writeDiagResultsXML(aOpMode);
    }


    //== utility programs copied over from vvLib.
    public void turnAbsoluteMxpGyroDegrees(vv_OpMode aOpMode, float fieldReferenceDegrees) throws InterruptedException {
        //clockwise is represented by clockwise numbers.
        //counterclockwise by negative angle numbers in degrees.
        //the fieldReferenceDegrees parameters measures degrees off the initial reference frame when the robot is started and the gyro is
        //calibrated.
        // >> IMPORTANT: This depends on the zIntegratedHeading not being altered by relative turns !!!

        //first take the absolute degrees and modulus down to 0 and 359.

        float targetDegrees = fieldReferenceDegrees % 360;

        //compare to the current gyro zIntegrated heading and store the result.
        //the Integrated zValue returned is positive for clockwise turns
        //read the heading and store it.

        float startingHeading = robot.getMxpGyroSensorHeading(aOpMode);
        float turnDegrees = targetDegrees - startingHeading;

        //make the turn using encoders

        if (DEBUG) {
            aOpMode.telemetryAddData("targetDegrees", "Value",
                    ":" + targetDegrees);
            aOpMode.telemetryAddData("Starting Z", "Value",
                    ":" + startingHeading);
            aOpMode.telemetryAddData("Turn Degrees", "Value",
                    ":" + turnDegrees);

            aOpMode.telemetryUpdate();
        }

        //optimize the turn, so that direction of turn results in smallest turn needed.

        if (Math.abs(turnDegrees) > 180) {
            turnDegrees = Math.signum(turnDegrees) * -1 * (360 - Math.abs(turnDegrees));
        }

        turnUsingEncoders(aOpMode, TURN_POWER, Math.abs(turnDegrees),
                turnDegrees > 0 ? vv_Constants.TurnDirectionEnum.Clockwise :
                        vv_Constants.TurnDirectionEnum.Counterclockwise);

        float finalDegrees = robot.getMxpGyroSensorHeading(aOpMode);
        Thread.sleep(50); //cooling off after gyro read to prevent error in next run.

        if (DEBUG) {
            aOpMode.telemetryAddData("New Bearing Degrees", "Value:",
                    ":" + finalDegrees);
            aOpMode.telemetryAddData("Turn Error Degrees", "Value:",
                    ":" + (targetDegrees - finalDegrees));
            aOpMode.telemetryUpdate();
        }

    }

    public void turnUsingEncoders(vv_OpMode aOpMode, float power, float angle, vv_Constants.TurnDirectionEnum TurnDirection)
            throws InterruptedException {

        //calculate the turn distance to be used in terms of encoder clicks.
        //for Andymark encoders.

        int turnDistance = (int) (2 * ((ROBOT_TRACK_DISTANCE) * angle
                * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION) /
                (MECCANUM_WHEEL_DIAMETER * 360));


        switch (TurnDirection) {
            case Clockwise:
                robot.runRobotToPosition(aOpMode, power, power, power, power, turnDistance, -turnDistance, turnDistance, -turnDistance, true);
                break;
            case Counterclockwise:
                robot.runRobotToPosition(aOpMode, power, power, power, power, -turnDistance, turnDistance, -turnDistance, turnDistance, true);
                break;
        }

        //wait just a bit for the commands to complete
        Thread.sleep(50);
    }

    public void universalMoveRobotPolar(vv_OpMode aOpMode, double polarAngle,
                                        double polarVelocity, double rotationalVelocity,
                                        long duration, vv_OpMode.StopCondition condition,
                                        boolean isPulsed, long pulseWidthDuration, long pulseRestDuration)
            throws InterruptedException {


        robot.universalMoveRobot(aOpMode, polarVelocity * Math.sin(Math.toRadians(polarAngle)),
                polarVelocity * Math.cos(Math.toRadians(polarAngle)), rotationalVelocity, duration, condition, isPulsed, pulseWidthDuration, pulseRestDuration);
    }

    enum ResultSeverity {CRITICAL, HIGH, MEDIUM, LOW, INFO, UNKNOWN}

    enum TestType {AUTOMATIC, MANUAL}

    public interface RunnableTest {
        boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException;
    }

    protected class RobotTest {
        String testElementName;
        int testElementId;
        String testShortDescription;
        String testLongDescription;
        boolean testResult;
        ResultSeverity testResultSeverity;
        String testResultMessage;
        String testRecommendation;
        long testTimeStamp;
        boolean testValidity; //validity of the test itself, set true if test initialized
        boolean testResultValidity; //validity of the results of the test, reset when running a new cycle.
        TestType testType;
        RunnableTest testRunMethod;

        public void setTestElementId(vv_OpMode aOpMode, int id) {
            testElementId = id;
        }

        public void setTestElementName(vv_OpMode aOpMode, String name) {
            testElementName = name;
        }

        public void setTestShortDescription(vv_OpMode aOpMode, String shortDescription) {
            testShortDescription = shortDescription;
        }

        public void setTestLongDescription(vv_OpMode aOpMode, String longDescription) {
            testLongDescription = longDescription;
        }

        public void setTestValidity(vv_OpMode aOpMode, boolean validity) {
            testValidity = validity;
        }

        public void setTestResultValidity(vv_OpMode aOpMode, boolean resultValidity) {
            testResultValidity = resultValidity;
        }

        public void setTestTimeStamp(vv_OpMode aOpMode, long timeStampMilliseconds) {
            testTimeStamp = timeStampMilliseconds;
        }

        public void setTestResult(vv_OpMode aOpMode, boolean result) {
            testResult = result;
        }

        public void setTestResultMessage(vv_OpMode aOpMode, String ResultMessage) {
            testResultMessage = ResultMessage;
        }

        public void setTestResultSeverity(vv_OpMode aOpMode, ResultSeverity resultSeverity) {
            testResultSeverity = resultSeverity;
        }

        public void setTestRecommendation(vv_OpMode aOpMode, String recommendation) {
            testRecommendation = recommendation;
        }

        public void setTestType(vv_OpMode aOpMode, TestType type) {
            testType = type;
        }

        public void setRunnableTest(vv_OpMode aOpMode, RunnableTest runnableTest) {
            testRunMethod = runnableTest;
        }

        public int getTestElementId(vv_OpMode aOpMode) {
            return testElementId;
        }

        public String getTestName(vv_OpMode aOpMode) {
            return testElementName;
        }

        public int getTestId(vv_OpMode aOpMode) {
            return testElementId;
        }

        public String getTestShortDescription(vv_OpMode aOpMode) {
            return testShortDescription;
        }

        public String getTestLongDescription(vv_OpMode aOpMode) {
            return testLongDescription;
        }

        public boolean getTestValidity(vv_OpMode aOpMode) {
            return testValidity;
        }

        public boolean getTestResultValidity(vv_OpMode aOpMode) {
            return testResultValidity;
        }

        public long getTestTimeStamp(vv_OpMode aOpMode) {
            return testTimeStamp;
        }

        public boolean getTestResult(vv_OpMode aOpMode) {
            return testResult;
        }

        public String getTestResultMessage(vv_OpMode aOpMode) {
            return testResultMessage;
        }

        public ResultSeverity getTestSeverity(vv_OpMode aOpMode) {
            return testResultSeverity;
        }

        public String getTestRecommendation(vv_OpMode aOpMode) {
            return testRecommendation;
        }

        public TestType getTestType(vv_OpMode aOpMode) {
            return testType;
        }

        public RunnableTest getTestRunnableTest(vv_OpMode aOpMode) {
            return testRunMethod;
        }

        public void createTest(vv_OpMode aOpMode, String name, int id, String shortDescription,
                               String longDescription, TestType testType, RunnableTest runnableTest) {

            //set values
            setTestElementId(aOpMode, id);
            setTestElementName(aOpMode, name);
            setTestShortDescription(aOpMode, shortDescription);
            setTestLongDescription(aOpMode, longDescription);
            setTestType(aOpMode, testType);
            setRunnableTest(aOpMode, runnableTest);
            setTestValidity(aOpMode, true);
            setTestResultValidity(aOpMode, false);
        }


    }

    class TestFrontLeftWheel implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {
            robot.setMotorMode(aOpMode, FRONT_LEFT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            robot.testMotor(aOpMode, FRONT_LEFT_MOTOR, 0.5f, 1000);
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);

            if (Math.abs(newMotorPosition - motorPosition)<MECCANUM_WHEEL_ENCODER_MARGIN) {
                //the motor encoder has not moved. we have a problem.
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed To Detect Front Left Wheel Rotation");
                robotTest.setTestRecommendation(aOpMode, "Please examine electrical and encoder connections");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Front Left Wheel");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestFrontRightWheel implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {
            robot.setMotorMode(aOpMode, FRONT_RIGHT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR);
            robot.testMotor(aOpMode, FRONT_RIGHT_MOTOR, 0.5f, 1000);
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR);
            if (Math.abs(newMotorPosition - motorPosition)<MECCANUM_WHEEL_ENCODER_MARGIN) {
                //the motor encoder has not moved. we have a problem.
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed To Detect Front Right Wheel Rotation");
                robotTest.setTestRecommendation(aOpMode, "Please examine electrical and encoder connections");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Front Right Wheel");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestBackLeftWheel implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {
            robot.setMotorMode(aOpMode, BACK_LEFT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
            int motorPosition = robot.getMotorPosition(aOpMode, BACK_LEFT_MOTOR);
            robot.testMotor(aOpMode, BACK_LEFT_MOTOR, 0.5f, 1000);
            int newMotorPosition = robot.getMotorPosition(aOpMode, BACK_LEFT_MOTOR);
            if (Math.abs(newMotorPosition - motorPosition)<MECCANUM_WHEEL_ENCODER_MARGIN) {
                //the motor encoder has not moved. we have a problem.
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed To Detect Back Left Wheel Rotation");
                robotTest.setTestRecommendation(aOpMode, "Please examine electrical and encoder connections");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Back Left Wheel");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestBackRightWheel implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {
            robot.setMotorMode(aOpMode, BACK_RIGHT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
            int motorPosition = robot.getMotorPosition(aOpMode, BACK_RIGHT_MOTOR);
            robot.testMotor(aOpMode, BACK_RIGHT_MOTOR, 0.5f, 1000);
            int newMotorPosition = robot.getMotorPosition(aOpMode, BACK_RIGHT_MOTOR);
            if (Math.abs(newMotorPosition - motorPosition)<MECCANUM_WHEEL_ENCODER_MARGIN) {
                //the motor encoder has not moved. we have a problem.
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed To Detect Back Right Wheel Rotation");
                robotTest.setTestRecommendation(aOpMode, "Please examine electrical and encoder");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Back Right Wheel");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestPlatformLeft implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {

            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); //representative motor
            float startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            robot.universalMoveRobotForDuration(aOpMode, -0.5f, 0.0f, 0.0f, 0); //move robot to left
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            float endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            if ((Math.abs(newMotorPosition-motorPosition)<MECCANUM_WHEEL_ENCODER_MARGIN) || (Math.abs(startingAngle - endingAngle) > 3)) {
                //the motor encoder has not moved. we have a problem.
                //or the platform has moved, but there is too much rotation
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed To Detect Platform Left Movement");
                robotTest.setTestRecommendation(aOpMode, "Please examine electrical and encoder connections");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Platform Left");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestPlatformRight implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {

            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); //representative motor
            float startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            robot.universalMoveRobotForDuration(aOpMode, 0.5f, 0.0f, 0.0f, 0); //move robot to left
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            float endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            if ((Math.abs(newMotorPosition-motorPosition)<MECCANUM_WHEEL_ENCODER_MARGIN)|| (Math.abs(startingAngle - endingAngle) > 3)) {
                //the motor encoder has not moved. we have a problem.
                //or the platform has moved, but there is too much rotation
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed To Detect Platform Right Movement");
                robotTest.setTestRecommendation(aOpMode, "Please examine electrical and encoder connections");

                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Platform Right");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestPlatformForward implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {

            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); //representative motor
            float startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            robot.universalMoveRobotForDuration(aOpMode, 0.0f, 0.5f, 0.0f, 0); //move robot to left
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            float endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            if ((Math.abs(newMotorPosition-motorPosition)<MECCANUM_WHEEL_ENCODER_MARGIN) || (Math.abs(startingAngle - endingAngle) > 3)) {
                //the motor encoder has not moved. we have a problem.
                //or the platform has moved, but there is too much rotation
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed To Detect Platform Forward Movement");
                robotTest.setTestRecommendation(aOpMode, "Please examine electrical and encoder connections");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Platform Forward");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestPlatformBackward implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {

            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); //representative motor
            float startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            robot.universalMoveRobotForDuration(aOpMode, 0.0f, -0.5f, 0.0f, 0); //move robot to left
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            float endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            if ((Math.abs(newMotorPosition-motorPosition)<MECCANUM_WHEEL_ENCODER_MARGIN) || (Math.abs(startingAngle - endingAngle) > 3)) {
                //the motor encoder has not moved. we have a problem.
                //or the platform has moved, but there is too much rotation
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed To Detect Platform Backward Movement");
                robotTest.setTestRecommendation(aOpMode, "Please examine electrical and encoder connections");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Platform Backward");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestPlatformRotation implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {


            float startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            turnAbsoluteMxpGyroDegrees(aOpMode, 90);
            float endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            float error = Math.abs((Math.abs(startingAngle - endingAngle) - 90));
            if (error > 3) {

                //We are off by more than 3 degrees in our gyro rotation
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed Platform Rotation Test"+"[Error:"+error+"]");
                if (error < 5) {

                    robotTest.setTestRecommendation(aOpMode,
                            "Please check MXP gyro and calibration, error >3 but <5 degrees");
                    robotTest.setTestResultSeverity(aOpMode, ResultSeverity.MEDIUM);
                }
                if ((error >= 5)&&(error<7)) {

                    robotTest.setTestRecommendation(aOpMode,
                            "Please check MXP gyro and calibration, error >5 degrees");
                    robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                }

                if (error >= 7) {

                    robotTest.setTestRecommendation(aOpMode,
                            "Please check MXP gyro and calibration, error >5 degrees");
                    robotTest.setTestResultSeverity(aOpMode, ResultSeverity.CRITICAL);
                }

                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Gyro Turns");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestRangeSensor implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {


            boolean rangeHasChanged = false;
            for (int degrees = 0; degrees < 360; degrees += 20) {
                double startingRange = robot.getUltrasonicDistance(aOpMode);
                turnAbsoluteMxpGyroDegrees(aOpMode, degrees);
                Thread.sleep(100);
                double endingRange = robot.getUltrasonicDistance(aOpMode);
                if (startingRange != endingRange) {
                    //Range has changed. The ultrasonic sensor is working
                    rangeHasChanged = true;
                    break;
                }
            }
            if (!rangeHasChanged) {
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed to detect change in Range Test");
                robotTest.setTestRecommendation(aOpMode,
                        "Please check Ultrasonic sensor and calibration, are there objects close by ?");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                return false;
            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Range Sensors");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestFloorSensor implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {


            boolean colorIntensityHasChanged = false;
            double startingColorIntensity = 0;
            double endingColorIntensity = 0;
            for (int degrees = 0; degrees < 360; degrees += 20) {
                startingColorIntensity = robot.getFloorColorIntensity(aOpMode);
                turnAbsoluteMxpGyroDegrees(aOpMode, degrees);
                Thread.sleep(100);
                endingColorIntensity = robot.getUltrasonicDistance(aOpMode);
                if (startingColorIntensity != endingColorIntensity) {
                    //Color has changed. The ultrasonic sensor is working
                    colorIntensityHasChanged = true;
                    break;
                }
            }
            if (!colorIntensityHasChanged) {
                if (startingColorIntensity != 0) {
                    robotTest.setTestResult(aOpMode, false);
                    robotTest.setTestResultValidity(aOpMode, true);
                    robotTest.setTestResultMessage(aOpMode, "Failed to detect change in floor color sensor");
                    robotTest.setTestRecommendation(aOpMode,
                            "Please check Floor Color Sensor connections and calibration");
                    robotTest.setTestResultSeverity(aOpMode, ResultSeverity.MEDIUM);
                    return false;
                } else {
                    robotTest.setTestResult(aOpMode, false);
                    robotTest.setTestResultValidity(aOpMode, true);
                    robotTest.setTestResultMessage(aOpMode, "Failed to detect non-zero Color Sensor Values");
                    robotTest.setTestRecommendation(aOpMode,
                            "Please check Floor Color Sensor connections and calibration");
                    robotTest.setTestResultSeverity(aOpMode, ResultSeverity.CRITICAL);
                    return false;
                }
            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);

            robotTest.setTestResultMessage(aOpMode, "No Problem with Floor Color Sensor");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestCapBall implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {

            int startingCapBallEncoderValue = 0;
            int endingCapBallEncoderValue = 0;

            try {
                startingCapBallEncoderValue = robot.getCapBallMotorEncoderPosition(aOpMode);
                //move the Cap Ball Lift by a small amount.

                robot.setCapBallPosition(aOpMode, startingCapBallEncoderValue + (CAP_BALL_POSITION_INCREMENT / 10));

                endingCapBallEncoderValue = robot.getCapBallMotorEncoderPosition(aOpMode);

                //return the lift to starting position
                robot.setCapBallPosition(aOpMode, startingCapBallEncoderValue);

            } catch (vv_Robot.MotorStalledException MSE) {
                aOpMode.telemetryAddData("Stalled During Test", "CapBall", "Lift");
                aOpMode.telemetryUpdate();
            }


            if (Math.abs(startingCapBallEncoderValue - endingCapBallEncoderValue)<CAP_BALL_ENCODER_MARGIN){
                //capBallHas not moved.
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed to detect Cap Ball Lift Movement");
                robotTest.setTestRecommendation(aOpMode,
                        "Please check CapBall Lift and calibration");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.CRITICAL);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Cap Ball Lift");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestChooChooTension implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {
            String failureReason=null;

            int startingChooChooTensionEncoderValue = 0;
            int endingChooChooTensionEncoderValue = 0;

            try {
                startingChooChooTensionEncoderValue = robot.getLauncherPowerPosition(aOpMode);
                //move the Choo Choo Tension by a small amount.

                robot.setLauncherPowerPosition(aOpMode, startingChooChooTensionEncoderValue + (LAUNCH_POWER_INCREMENT));
                endingChooChooTensionEncoderValue = robot.getLauncherPowerPosition(aOpMode);

                //return the Choo Choo Tension to starting position
                robot.setLauncherPowerPosition(aOpMode, startingChooChooTensionEncoderValue);


            } catch (vv_Robot.MotorStalledException MSE) {
                aOpMode.telemetryAddData("Stalled During Test", "Choo Choo Tensioner", " Choo Choo Arm");
                failureReason="Stalled During Test";

                aOpMode.telemetryUpdate();
            }


            if (Math.abs(startingChooChooTensionEncoderValue - endingChooChooTensionEncoderValue)<WORM_DRIVE_ENCODER_MARGIN ) {
                //Choo Choo Tension Has not moved.
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode,
                        "Failed to detect Choo Choo Arm Tension changes"+
                                "[Failure Reason:"+ failureReason+"]"+
                                "["+"Start E Val:"+startingChooChooTensionEncoderValue+"]"+
                                "["+"Ending E Val:"+endingChooChooTensionEncoderValue+"]");



                robotTest.setTestRecommendation(aOpMode,
                        "Please check Choo Choo Tension and calibration");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Choo Choo Tension");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestChooChooLaunch implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {

            int startingChooChooLaunchEncoderValue = 0;
            int endingChooChooLaunchEncoderValue = 0;

            startingChooChooLaunchEncoderValue = robot.getChooChooArmEncoderPosition(aOpMode);
            //move the Choo Choo Tension by a small amount.

            robot.setChooChooArmEncoderPosition(aOpMode, startingChooChooLaunchEncoderValue + LAUNCH_POSITION_INCREMENT);
            endingChooChooLaunchEncoderValue = robot.getChooChooArmEncoderPosition(aOpMode);

            //return the Choo Choo Tension to starting position
            robot.setChooChooArmEncoderPosition(aOpMode, startingChooChooLaunchEncoderValue);



            if (Math.abs(startingChooChooLaunchEncoderValue - endingChooChooLaunchEncoderValue)<ARM_MOTOR_ENCODER_MARGIN ){
                //Choo Choo Launcher Has not moved.
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed to detect Choo Choo Launch Encoder Changes");
                robotTest.setTestRecommendation(aOpMode,
                        "Please check Choo Choo Launcher Arm and calibration");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Choo Choo Launch Arm");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestIntake implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {

            int startingIntakeEncoderValue = 0;
            int endingIntakeEncoderValue = 0;


            robot.setMotorMode(aOpMode, INTAKE_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);
            startingIntakeEncoderValue = robot.getIntakeEncoderPosition(aOpMode);
            //move the Intake rotation by a small amount.

            robot.setIntakeEncoderPosition(aOpMode, startingIntakeEncoderValue + INTAKE_INCREMENT);
            endingIntakeEncoderValue = robot.getIntakeEncoderPosition(aOpMode);

            //return the Intake to starting position
            robot.setIntakeEncoderPosition(aOpMode, startingIntakeEncoderValue);

            robot.setMotorMode(aOpMode, INTAKE_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);

            if (Math.abs(startingIntakeEncoderValue - endingIntakeEncoderValue)<CAP_BALL_ENCODER_MARGIN) {
                //Intake Has not moved.
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed to detect Intake Encoder Changes");
                robotTest.setTestRecommendation(aOpMode,
                        "Please check Intake connections and calibration");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Intake");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;
        }
    }

    class TestPlatformDiagonals implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {

            //we will move the robot in 4 phases to test all 4 diagonals.
            //We will pick representative motors based on the specific diagonal as all
            //wheels will not rotate with power during diagonal moves.
            //we also check for greater than 3 degree rotation as a means to check if there is platform
            //rotation occuring during these moves. If the platform is rotating during these moves, this will
            //lead to innacuracy and may be symptomatic of other underlying problems such as
            //irregular weight distribution and lack of coplanar 4 wheel contact.
            //this code can be refactored to removed repetitive code.

            //phase 1: Move Robot 45 degrees.
            boolean failedDiagTopRight = false;
            boolean failedDiagTopRightRotation = false;

            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); //representative motor
            float startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            universalMoveRobotPolar(aOpMode, 45, 0.5f, 0.0f, 750, new falseCondition(), false, 0, 0); //move robot diag 45
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            float endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            float rotationDiagTopRightAngle = Math.abs(startingAngle - endingAngle);

            if (newMotorPosition == motorPosition) {
                //the motor encoder has not moved. we have a problem.
                failedDiagTopRight = true;
            }
            if (rotationDiagTopRightAngle > 3) {
                failedDiagTopRightRotation = true;
            }

            //phase 2: Move Robot 135 degrees
            boolean failedDiagBottomRight = false;
            boolean failedDiagBottomRightRotation = false;

            motorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR); //representative motor
            startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            universalMoveRobotPolar(aOpMode, 135, 0.5f, 0.0f, 750, new falseCondition(), false, 0, 0); //move robot diag 45
            newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR);
            endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            float rotationDiagBottomRightAngle = Math.abs(startingAngle - endingAngle);

            if (newMotorPosition == motorPosition) {
                //the motor encoder has not moved. we have a problem.
                failedDiagBottomRight = true;
            }
            if (rotationDiagBottomRightAngle > 3) {
                failedDiagBottomRightRotation = true;
            }

            //phase 3: Move Robot -45 degrees
            boolean failedDiagTopLeft = false;
            boolean failedDiagTopLeftRotation = false;

            motorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR); //representative motor
            startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            universalMoveRobotPolar(aOpMode, -45, 0.5f, 0.0f, 750, new falseCondition(), false, 0, 0); //move robot diag 45
            newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR);
            endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            float rotationDiagTopLeftAngle = Math.abs(startingAngle - endingAngle);

            if (newMotorPosition == motorPosition) {
                //the motor encoder has not moved. we have a problem.
                failedDiagTopLeft = true;
            }
            if (rotationDiagTopLeftAngle > 3) {
                failedDiagTopLeftRotation = true;
            }


            //phase 4: Move Robot -135 degrees
            boolean failedDiagBottomLeft = false;
            boolean failedDiagBottomLeftRotation = false;

            motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); //representative motor
            startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            universalMoveRobotPolar(aOpMode, -135, 0.5f, 0.0f, 750, new falseCondition(), false, 0, 0); //move robot diag 45
            newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            float rotationDiagBottomLeftAngle = Math.abs(startingAngle - endingAngle);




            if (newMotorPosition == motorPosition) {
                //the motor encoder has not moved. we have a problem.
                failedDiagBottomLeft = true;
            }

            if (rotationDiagBottomLeftAngle > 3) {
                failedDiagBottomLeftRotation = true;
            }



            if (failedDiagTopLeft || failedDiagTopRight || failedDiagBottomLeft || failedDiagBottomRight) {
                //failed these tests
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode, "Failed to detect Platform Diagonal movement");
                robotTest.setTestRecommendation(aOpMode, "Please check Electrical and Encoder connections to wheel motors");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.CRITICAL);
                return false;
            }

            if (failedDiagTopLeftRotation ||
                    failedDiagTopRightRotation ||
                    failedDiagBottomLeftRotation ||
                    failedDiagBottomRightRotation) {
                //we have too much rotation
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestResultMessage(aOpMode,
                        "Detected too much rotation when performing platform diagonal movements"+
                "[TDLR:"+rotationDiagTopLeftAngle+"]"+
                "[TDRR:"+rotationDiagTopRightAngle+"]"+
                "[BDLR:"+rotationDiagBottomLeftAngle+"]"+
                "[BDRR:"+rotationDiagBottomRightAngle+"]");
                robotTest.setTestRecommendation(aOpMode, "Check weight distribution or wheel coplanarity");
                robotTest.setTestResultSeverity(aOpMode, ResultSeverity.MEDIUM);
                return false;
            }

            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestResultMessage(aOpMode, "No Problem with Platform Diagonals");
            robotTest.setTestRecommendation(aOpMode, "No Recommendation");
            robotTest.setTestResultSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    public class falseCondition implements vv_OpMode.StopCondition {
        //can be used as an empty condition, so the robot keeps running in universal movement
        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
            return (false);
        }
    }


}
