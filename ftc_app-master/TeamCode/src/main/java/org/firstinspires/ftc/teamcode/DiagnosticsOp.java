package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.w3c.dom.Document;
import org.w3c.dom.NodeList;

import java.io.File;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.GregorianCalendar;
import java.util.LinkedHashMap;
import java.util.Map;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpression;
import javax.xml.xpath.XPathFactory;

/**
 * Created by Rahul on 11/27/2016.
 */

@TeleOp
public class DiagnosticsOp extends vv_OpMode {

	private static final String LOG_TAG = "DiagnosticsOp";
	private static final String INPUT_TELEMETRY_KEY = "Input";
	private static final String OUTPUT_TELEMETRY_KEY = "Output";
	private static final int WHEEL_POWER = 50;
	private static final int WHEEL_DISTANCE = 15; // Centimeters
	private static final int WHEEL_TIME = 2000; // milliseconds
	private static final int TOUCH_WAIT_TIME = 5000; // milliseconds
	private static final int INPUT_WAIT_TIME = 1000; // milliseconds
	private vv_Lib robotLibrary;
	private LinkedHashMap<String, ChoiceRecord> choices;

	@Override
	public void runOpMode() throws InterruptedException {
		telemetryAddData(LOG_TAG, OUTPUT_TELEMETRY_KEY, "Initializing...");
		telemetryUpdate();
		initialize();
		telemetryAddData(LOG_TAG, OUTPUT_TELEMETRY_KEY, "Done with initialization");
		telemetryUpdate();
		waitForStart();


		// Go through all choices
		for (Map.Entry<String, ChoiceRecord> choicesEntry : choices.entrySet()) {
			if (choicesEntry.getValue().getTestMotor()) {   // Check if we need to do the test through XML file
				if (getUserConfirmation(String.format("Test the %s? A for yes; B for no",
						choicesEntry.getKey()))) {   // Check if user wants to do test
					callTestElementMethod(choicesEntry);
				}
				gamepadInputWait();
			}
		}

		// Print all errors
		for (Map.Entry<String, ChoiceRecord> choicesEntry : choices.entrySet()) {
			if (choicesEntry.getValue().getErrorStatus()) {
				printElementError(choicesEntry);

				getUserConfirmation("Press A or B to continue");    // Wait until user reads it completely
				gamepadInputWait(); // Wait for a bit so gamepad input does not get read twice
			}
		}
	}

	private void initialize() throws InterruptedException {
		robotLibrary = new vv_Lib(this);
		choices = getTests();
	}

	private LinkedHashMap<String, ChoiceRecord> getTests() {
		LinkedHashMap<String, ChoiceRecord> formattedChoices = new LinkedHashMap<>();
		LinkedHashMap<String, Boolean> xmlChoices;

		// Parse XML and get hashmap
		XmlParser xmlParser = new XmlParser();
		xmlChoices = xmlParser.getChoicesMap();

		for (String key : xmlChoices.keySet()) {
			// Take xml's hashmap and convert it into the one we want
			formattedChoices.put(key, new ChoiceRecord(xmlChoices.get(key)));
		}

		return formattedChoices;
	}

	private void callTestElementMethod(Map.Entry<String, ChoiceRecord> choicesEntry) {
		try {
			Method method = DiagnosticsOp.class.getDeclaredMethod(choicesEntry.getKey());
			telemetryAddData(LOG_TAG, OUTPUT_TELEMETRY_KEY, "Running " + choicesEntry.getKey());
			telemetryUpdate();
			choicesEntry.getValue().setErrorStatus((Boolean) method.invoke(this));
		} catch (NoSuchMethodException e) {
			Log.e(LOG_TAG, e.getMessage());
			choicesEntry.getValue().setErrorStatus(true);
			choicesEntry.getValue().addErrorMessage("Could not find method: " + choicesEntry.getKey());
		} catch (InvocationTargetException e) {
			if (e.getCause() instanceof InterruptedException) {
				Log.e(LOG_TAG, e.getMessage());
				choicesEntry.getValue().setErrorStatus(true);
				choicesEntry.getValue().addErrorMessage("Interrupted Exception: " + choicesEntry.getKey());
			} else {
				Log.e(LOG_TAG, e.getMessage());
				choicesEntry.getValue().setErrorStatus(true);
				choicesEntry.getValue().addErrorMessage("Could invoke method: " + choicesEntry.getKey());
			}
		} catch (IllegalAccessException e) {
			Log.e(LOG_TAG, e.getMessage());
			choicesEntry.getValue().setErrorStatus(true);
			choicesEntry.getValue().addErrorMessage("Could not access method: " + choicesEntry.getKey());
		} catch (NullPointerException e) {
			Log.e(LOG_TAG, e.getMessage());
			choicesEntry.getValue().setErrorStatus(true);
			choicesEntry.getValue().addErrorMessage("Method did not return boolean value; Method may be empty");
		}
	}

	private void printElementError(Map.Entry<String, ChoiceRecord> choicesEntry) {
		telemetryAddData(LOG_TAG, "Test failed", choicesEntry.getKey());

		// Print all error messages
		int errorMessageCount = 0;
		for (String errorMessage : choicesEntry.getValue().getErrorMessages()) {
			// Must have different key each time or line with same key will be overwritten
			telemetryAddData(LOG_TAG, "Reason" + errorMessageCount++, errorMessage);
		}

		// If there are no messages, the user said the robot element wasn't working; it wasn't a fault with this program
		if (errorMessageCount == 0) {
			telemetryAddData(LOG_TAG, "Reason", "User said it wasn't working");
		}
	}

	private boolean getUserConfirmation(String message) {
		// Clear screen and print message
		telemetryAddData(LOG_TAG, INPUT_TELEMETRY_KEY, message);
		telemetryUpdate();
		// Wait until A or B is pressed
		while (!gamepad1.a && !gamepad1.b) ;

		// If A is pressed, return yes. Otherwise, return no
		return gamepad1.a;
	}

	private boolean didItRun(String testName) {
		telemetryAddData(LOG_TAG, INPUT_TELEMETRY_KEY,
				String.format("Did %s work? A for yes; B for no", testName));
		telemetryUpdate();
		// Wait until A or B is pressed
		while (!gamepad1.a && !gamepad1.b) ;

		// If A is pressed, return yes. Otherwise, return no
		return gamepad1.a;
	}

	private void gamepadInputWait() throws InterruptedException {
		telemetryAddData(LOG_TAG, OUTPUT_TELEMETRY_KEY, "Waiting...");
		telemetryUpdate();
		Thread.sleep(INPUT_WAIT_TIME);
	}

	// Motors
	private boolean frontrightwheel() throws InterruptedException {
		robotLibrary.runAllMotors(this, 0, WHEEL_POWER, 0, 0);
		Thread.sleep(WHEEL_TIME);
		robotLibrary.stopAllMotors(this);

		return !didItRun(new Object(){}.getClass().getEnclosingMethod().getName());
	}

	private boolean frontleftwheel() throws InterruptedException {
		robotLibrary.runAllMotors(this, WHEEL_POWER, 0, 0, 0);
		Thread.sleep(WHEEL_TIME);
		robotLibrary.stopAllMotors(this);

		return !didItRun(new Object(){}.getClass().getEnclosingMethod().getName());
	}


	/*
	 * Robot element testing methods
	 * The names are made to be the same as the tag in the XML file, so they do not follow camel-casing
	 */

	private boolean backrightwheel() throws InterruptedException {
		robotLibrary.runAllMotors(this, 0, 0, 0, WHEEL_POWER);
		Thread.sleep(WHEEL_TIME);
		robotLibrary.stopAllMotors(this);

		return !didItRun(new Object(){}.getClass().getEnclosingMethod().getName());
	}

	private boolean backleftwheel() throws InterruptedException {
		robotLibrary.runAllMotors(this, 0, 0, WHEEL_POWER, 0);
		Thread.sleep(WHEEL_TIME);
		robotLibrary.stopAllMotors(this);

		return !didItRun(new Object(){}.getClass().getEnclosingMethod().getName());
	}

	private boolean forwards() throws InterruptedException {
		robotLibrary.moveWheels(this, WHEEL_DISTANCE, WHEEL_POWER, vv_Constants.DirectionEnum.Forward, false);
		return !didItRun(new Object(){}.getClass().getEnclosingMethod().getName());
	}

	private boolean backwards() throws InterruptedException {
		robotLibrary.moveWheels(this, WHEEL_DISTANCE, WHEEL_POWER, vv_Constants.DirectionEnum.Backward, false);
		return !didItRun(new Object(){}.getClass().getEnclosingMethod().getName());
	}

	private boolean sidewaysright() throws InterruptedException {
		robotLibrary.moveWheels(this, WHEEL_DISTANCE, WHEEL_POWER, vv_Constants.DirectionEnum.SidewaysRight, false);
		return !didItRun(new Object(){}.getClass().getEnclosingMethod().getName());
	}

	private boolean sidewaysleft() throws InterruptedException {
		robotLibrary.moveWheels(this, WHEEL_DISTANCE, WHEEL_POWER, vv_Constants.DirectionEnum.SidewaysLeft, false);
		return !didItRun(new Object(){}.getClass().getEnclosingMethod().getName());
	}

	private boolean wormdrive() throws InterruptedException, vv_Robot.MotorStalledException {
		Calendar cal = new GregorianCalendar();
		cal.setTimeInMillis(System.currentTimeMillis());
		while(System.currentTimeMillis() - cal.getTimeInMillis() > 1000) {
			robotLibrary.increaseLauncherPower(this);
		}

		Thread.sleep(2000);

		cal.setTimeInMillis(System.currentTimeMillis());
		while (System.currentTimeMillis() - cal.getTimeInMillis() > 1000) {
			robotLibrary.decreaseLauncherPower(this);
		}

		return !didItRun(new Object(){}.getClass().getEnclosingMethod().getName());
	}

	private boolean launcher() throws InterruptedException, vv_Robot.MotorStalledException {
		robotLibrary.shootBall(this);
		return !didItRun(new Object(){}.getClass().getEnclosingMethod().getName());
	}

	private boolean intake() throws InterruptedException {
		robotLibrary.toggleIntake(this);
		Thread.sleep(2000);
		robotLibrary.toggleIntake(this);

		return !didItRun(new Object(){}.getClass().getEnclosingMethod().getName());
	}

	private void lift() {
	}

	// Servos
	private void intakegate() {
	}

	private void beacon() {
	}

	private void capball() {
	}

	// Sensors
	private void floorcolor() throws InterruptedException {

	}

	private boolean beaconcolor() throws InterruptedException {
		Calendar cal = new GregorianCalendar();
		cal.setTimeInMillis(System.currentTimeMillis());

		while (System.currentTimeMillis() - cal.getTimeInMillis() > 5000) {
			robotLibrary.showBeaconLeftColorValuesOnTelemetry(this, true);
		}

		return !didItRun(new Object(){}.getClass().getEnclosingMethod().getName());
	}

	private boolean launcherlimittouch() throws InterruptedException {
		Calendar cal = new GregorianCalendar();
		cal.setTimeInMillis(System.currentTimeMillis());

		// Wait until sensor is touched or 5 seconds have passed
//		while (!robotLibrary.isArmAtLimit(this) && (System.currentTimeMillis() - cal.getTimeInMillis() < TOUCH_WAIT_TIME));

		return true;	//TODO: FIX
	}

	private void liftlimittouch() {
	}

	private boolean ultrasonic() throws InterruptedException {
//		double readings[] = {0};
//
//		while (gamepad1.a || gamepad1.b) {
//			for (int i = 0, j = 0; i < 3 && j < 10; i++, j++) {
//				readings[i] = robotLibrary.getFloorUltrasonicReading(this);
//				//wait between readings
//				Thread.sleep(20);
//				if (readings[i] == 0) {
//					i--; //bad read, redo. to a maximum of 10 reads
//				}
//			}
//
//			Arrays.sort(readings);
//			telemetryAddData("Ultrasonic", "Readings", String.valueOf(readings[1]));
//		}

		return !gamepad1.a;
	}

	private boolean gyro() throws InterruptedException {
		robotLibrary.turnAbsoluteMxpGyroDegrees(this, 180);

		return !didItRun(new Object(){}.getClass().getEnclosingMethod().getName());
	}

	private static class ChoiceRecord {
		private boolean testMotor;
		private boolean errorStatus;
		private ArrayList<String> errorMessages;

		public ChoiceRecord(boolean testMotor) {
			this.testMotor = testMotor;
			errorStatus = false;
			errorMessages = new ArrayList<>();
		}

		public void addErrorMessage(String errorMessage) {
			errorMessages.add(errorMessage);
		}

		public boolean getTestMotor() {
			return testMotor;
		}

		public boolean getErrorStatus() {
			return errorStatus;
		}

		public void setErrorStatus(boolean errorStatus) {
			this.errorStatus = errorStatus;
		}

		public ArrayList<String> getErrorMessages() {
			return errorMessages;
		}
	}

	// Code is based off of AutoXMLParser:
	// https://gist.github.com/rsquared226/21cf8b0d3e3476b38f22982f698d0388
	private static class XmlParser {
		private static final String FILE_NAME = Environment.getExternalStorageDirectory().getPath() +
				"/PACAR/DiagChoices.xml";
		private static final String LOG_TAG = "XmlParser";

		private LinkedHashMap<String, Boolean> choicesMap;

		public XmlParser() {
			choicesMap = new LinkedHashMap<>();
			parseXml();
		}

		private void parseXml() {
			try {
				File xmlFile = new File(FILE_NAME);
				DocumentBuilderFactory documentBuilderFactory = DocumentBuilderFactory.newInstance();
				DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();

				// Attempt to parse XML into DOM structure, initializing document
				Document document = documentBuilder.parse(xmlFile);

				// Find choices in XML DOM using XPATH
				XPath xPath = XPathFactory.newInstance().newXPath();

				// xpression is used to identify nodes. Needs to be compiled and stored
				// Retrieve all elements that are children of Choices
				// TODO: Change root node in app. For now, node is AutoChoices because I forgot to change it
				XPathExpression expression = xPath.compile("/AutoChoices/*");

				NodeList nodes = (NodeList) expression.evaluate(document, XPathConstants.NODESET);

				for (int i = 0; i < nodes.getLength(); i++) {
					choicesMap.put(nodes.item(i).getNodeName().toLowerCase(), Boolean.parseBoolean(nodes.item(i).getTextContent()));
				}
			} catch (Exception e) {
				Log.e(LOG_TAG, e.getMessage());
				e.printStackTrace();
			}
		}

		public LinkedHashMap<String, Boolean> getChoicesMap() {
			return choicesMap;
		}
	}
}
