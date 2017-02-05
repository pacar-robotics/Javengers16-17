package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_RIGHT_MOTOR;

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
		robotTestArray = new RobotTest[20];

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
		for (int i = 0; i < robotTestArray.length; i++) {
			if ((robotTestArray[i].getTestValidity(aOpMode)) &&
					(robotTestArray[i].getTestType(aOpMode) == TestType.AUTOMATIC)) {
				//runnable test, it has been initialized and it is an automatic test

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

	}

	public void analyzeTestResults(vv_OpMode aOpMode) throws InterruptedException {
		boolean noErrorsFound = true;

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
						Thread.sleep(2000);

					}

				}

			}

		}
	}


	enum ResultSeverity {CRITICAL, HIGH, MEDIUM, LOW, INFO}


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

		public void setTestSeverity(vv_OpMode aOpMode, ResultSeverity resultSeverity) {
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
			Thread.sleep(50); //to complete action.
			int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
			robot.testMotor(aOpMode, FRONT_LEFT_MOTOR, 0.5f, 1000);
			int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
			if (newMotorPosition == motorPosition) {
				//the motor encoder has not moved. we have a problem.
				robotTest.setTestResult(aOpMode, false);
				robotTest.setTestResultValidity(aOpMode, true);
				robotTest.setTestRecommendation(aOpMode, "Please examine electrical connections, front left motor or encoder failed");
				robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
				return false;

			}
			//there is no problem.
			robotTest.setTestResult(aOpMode, true);
			robotTest.setTestResultValidity(aOpMode, true);
			robotTest.setTestRecommendation(aOpMode, "No problem with Front Left Wheel");
			robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
			return true;

		}
	}

	class TestFrontRightWheel implements RunnableTest {
		public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {
			robot.setMotorMode(aOpMode, FRONT_RIGHT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
			Thread.sleep(50); //to complete action.
			int motorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR);
			robot.testMotor(aOpMode, FRONT_RIGHT_MOTOR, 0.5f, 1000);
			int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR);
			if (newMotorPosition == motorPosition) {
				//the motor encoder has not moved. we have a problem.
				robotTest.setTestResult(aOpMode, false);
				robotTest.setTestResultValidity(aOpMode, true);
				robotTest.setTestRecommendation(aOpMode, "Please examine electrical connections, front right motor or encoder failed");
				robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
				return false;

			}
			//there is no problem.
			robotTest.setTestResult(aOpMode, true);
			robotTest.setTestResultValidity(aOpMode, true);
			robotTest.setTestRecommendation(aOpMode, "No problem with Front Right Wheel");
			robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
			return true;

		}
	}

	class TestBackLeftWheel implements RunnableTest {
		public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {
			robot.setMotorMode(aOpMode, BACK_LEFT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
			Thread.sleep(50); //to complete action.
			int motorPosition = robot.getMotorPosition(aOpMode, BACK_LEFT_MOTOR);
			robot.testMotor(aOpMode, BACK_LEFT_MOTOR, 0.5f, 1000);
			int newMotorPosition = robot.getMotorPosition(aOpMode, BACK_LEFT_MOTOR);
			if (newMotorPosition == motorPosition) {
				//the motor encoder has not moved. we have a problem.
				robotTest.setTestResult(aOpMode, false);
				robotTest.setTestResultValidity(aOpMode, true);
				robotTest.setTestRecommendation(aOpMode, "Please examine electrical connections, back left motor or encoder failed");
				robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
				return false;

			}
			//there is no problem.
			robotTest.setTestResult(aOpMode, true);
			robotTest.setTestResultValidity(aOpMode, true);
			robotTest.setTestRecommendation(aOpMode, "No problem with Back Left Wheel");
			robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
			return true;

		}
	}

	class TestBackRightWheel implements RunnableTest {
		public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {
			robot.setMotorMode(aOpMode, BACK_RIGHT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
			Thread.sleep(50); //to complete action.
			int motorPosition = robot.getMotorPosition(aOpMode, BACK_RIGHT_MOTOR);
			robot.testMotor(aOpMode, BACK_RIGHT_MOTOR, 0.5f, 1000);
			int newMotorPosition = robot.getMotorPosition(aOpMode, BACK_LEFT_MOTOR);
			if (newMotorPosition == motorPosition) {
				//the motor encoder has not moved. we have a problem.
				robotTest.setTestResult(aOpMode, false);
				robotTest.setTestResultValidity(aOpMode, true);
				robotTest.setTestRecommendation(aOpMode, "Please examine electrical connections, Back Right motor or encoder failed");
				robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
				return false;

			}
			//there is no problem.
			robotTest.setTestResult(aOpMode, true);
			robotTest.setTestResultValidity(aOpMode, true);
			robotTest.setTestRecommendation(aOpMode, "No problem with Back Right Wheel");
			robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
			return true;

		}
	}

}
