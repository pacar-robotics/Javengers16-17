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
	private LinkedHashMap<String, ChoiceRecord> choices;

	private static final String LOG_TAG = "DiagnosticsOp";
	private static final String INPUT_TELEMETRY_KEY = "Input";
	private static final String OUTPUT_TELEMETRY_KEY = "Output";
	private static final int INPUT_WAIT_TIME = 1000; // milliseconds

	@Override
	public void runOpMode() throws InterruptedException {
		initialize();
		waitForStart();


		// Go through all choices
		for (Map.Entry<String, ChoiceRecord> choicesEntry : choices.entrySet()) {
			if (choicesEntry.getValue().getTestMotor()) {   // Check if we need to do the test through XML file
				callTestElementMethod(choicesEntry);
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
		telemetryAddData(LOG_TAG, OUTPUT_TELEMETRY_KEY, "Initializing...");
		telemetryUpdate();

		choices = getTests();

		telemetryAddData(LOG_TAG, OUTPUT_TELEMETRY_KEY, "Done with initialization");
		telemetryUpdate();
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
			Method method = vv_DiagLib.class.getDeclaredMethod(choicesEntry.getKey());
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

	private void gamepadInputWait() throws InterruptedException {
		telemetryAddData(LOG_TAG, OUTPUT_TELEMETRY_KEY, "Waiting...");
		telemetryUpdate();
		Thread.sleep(INPUT_WAIT_TIME);
	}
}
