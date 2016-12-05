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

		public void setErrorStatus(boolean errorStatus) {
			this.errorStatus = errorStatus;
		}

		public boolean getTestMotor() {
			return testMotor;
		}

		public boolean getErrorStatus() {
			return errorStatus;
		}

		public ArrayList<String> getErrorMessages() {
			return errorMessages;
		}
	}

	// Code is based off of AutoXMLParser:
	// https://gist.github.com/rsquared226/21cf8b0d3e3476b38f22982f698d0388
	private static class XmlParser {
		private final String FILE_NAME = Environment.getExternalStorageDirectory().getPath() +
				"/PACAR/AutoChoices.xml";
		private final String LOG_TAG = "XmlParser";

		private LinkedHashMap<String, Boolean> choicesMap;

		public XmlParser() {
			choicesMap = parseXml();
		}

		private LinkedHashMap<String, Boolean> parseXml() {
			LinkedHashMap<String, Boolean> choicesMap = new LinkedHashMap<>();
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
					choicesMap.put(nodes.item(i).getNodeName(), Boolean.valueOf(nodes.item(i).getTextContent()));
				}

			} catch (Exception e) {
				Log.e(LOG_TAG, e.getMessage());
				return null;
			}

			return choicesMap;
		}

		public LinkedHashMap<String, Boolean> getChoicesMap() {
			return choicesMap;
		}
	}

	private vv_Lib robotLibrary;
	private LinkedHashMap<String, ChoiceRecord> choices;

	private static final String LOG_TAG = "DiagnosticsOp";
	private static final String INPUT_TELEMETRY_MESSAGE = "input: ";
	private static final int WHEEL_POWER = 50;
	private static final int WHEEL_DISTANCE = 15; // Centimeters
	private static final int WHEEL_TIME = 2000; // milliseconds

	@Override
	public void runOpMode() throws InterruptedException {
		initialize();

		// Go through all choices
		for (Map.Entry<String, ChoiceRecord> choicesEntry : choices.entrySet()) {
			// First check if we need to do the test through XML file, then ask the user
			if (choicesEntry.getValue().getTestMotor() && getUserConfirmation(choicesEntry.getKey())) {
				callTestElementMethod(choicesEntry);
			}
		}

		// Print all errors
		for (Map.Entry<String, ChoiceRecord> choicesEntry : choices.entrySet()) {
			if (choicesEntry.getValue().getErrorStatus()) {
				printElementError(choicesEntry);
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
		}
	}

	private void printElementError(Map.Entry<String, ChoiceRecord> choicesEntry) {
		telemetryUpdate();
		telemetryAddData(LOG_TAG, "Test failed", choicesEntry.getKey());

		// Print all error messages
		int errorMessageCount = 0;
		for (String errorMessage : choicesEntry.getValue().getErrorMessages()) {
			// Must have different key each time or line with same key will be overwritten
			telemetryAddData(LOG_TAG, "Reason" + errorMessageCount++, errorMessage);
		}
	}

	private boolean getUserConfirmation(String testName) {
		// Clear screen and print message
		telemetryUpdate();
		telemetryAddData(LOG_TAG, INPUT_TELEMETRY_MESSAGE,
				String.format("Test the %s? A for yes; B for no", testName));

		// Wait until A or B is pressed
		while (!gamepad1.a && !gamepad1.b);

		// If A is pressed, return yes. Otherwise, return no
		return gamepad1.a;
	}


	/*
	 * Robot element testing methods
	 * The names are made to be the same as the tag in the XML file, so they do not follow camel-casing
	 */

	// TODO 12/3/2016: Fill in method stubs

	// Motors
	private void frontrightwheel() throws InterruptedException {
		robotLibrary.runAllMotors(this, 0, WHEEL_POWER, 0, 0);
		Thread.sleep(WHEEL_TIME);
		robotLibrary.stopAllMotors(this);
	}

	private void frontleftwheel() throws InterruptedException {
		robotLibrary.runAllMotors(this, WHEEL_POWER, 0, 0, 0);
		Thread.sleep(WHEEL_TIME);
		robotLibrary.stopAllMotors(this);
	}

	private void backrightwheel() throws InterruptedException {
		robotLibrary.runAllMotors(this, 0, 0, 0, WHEEL_POWER);
		Thread.sleep(WHEEL_TIME);
		robotLibrary.stopAllMotors(this);
	}

	private void backleftwheel() throws InterruptedException {
		robotLibrary.runAllMotors(this, 0, 0, WHEEL_POWER, 0);
		Thread.sleep(WHEEL_TIME);
		robotLibrary.stopAllMotors(this);
	}

	private void forwards() throws InterruptedException {
		robotLibrary.moveWheels(this, WHEEL_DISTANCE, WHEEL_POWER, vv_Constants.DirectionEnum.Forward);
	}

	private void backwards() throws InterruptedException {
		robotLibrary.moveWheels(this, WHEEL_DISTANCE, WHEEL_POWER, vv_Constants.DirectionEnum.Backward);
	}

	private void sidewaysright() throws InterruptedException {
		robotLibrary.moveWheels(this, WHEEL_DISTANCE, WHEEL_POWER, vv_Constants.DirectionEnum.SidewaysRight);
	}

	private void sidwaysleft() throws InterruptedException {
		robotLibrary.moveWheels(this, WHEEL_DISTANCE, WHEEL_POWER, vv_Constants.DirectionEnum.SidewaysLeft);
	}

	private void wormdrive() {
	}

	private void launcher() {
	}

	private void intake() {
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
	private void floorcolor() {
	}

	private void beacontouch() {
	}

	private void beaconcolor() {
	}

	private void launcherlimittouch() {
	}

	private void liftlimittouch() {
	}

	private void ultrasonic() {
	}

	private void gyro() {
	}
}
