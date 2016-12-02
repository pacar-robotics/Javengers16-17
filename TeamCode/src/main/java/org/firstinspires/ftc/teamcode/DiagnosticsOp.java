package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import android.util.Log;

import org.w3c.dom.Document;
import org.w3c.dom.NodeList;

import java.io.File;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
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

		private HashMap<String, Boolean> choicesMap;

		public XmlParser() {
			choicesMap = parseXml();
		}

		private HashMap<String, Boolean> parseXml() {
			HashMap<String, Boolean> choicesMap = new HashMap<>();
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

		public HashMap<String, Boolean> getChoicesMap() {
			return choicesMap;
		}
	}

	private vv_Lib robotLibrary;
	private HashMap<String, ChoiceRecord> choices;

	private static final String LOG_TAG = "DiagnosticsOp";
	private static final String INPUT_TELEMETRY_MESSAGE = "input: ";

	@Override
	public void runOpMode() throws InterruptedException {
		initialize();
		choices = getTests();

		// Go through all choices
		for (Map.Entry<String, ChoiceRecord> choicesEntry : choices.entrySet()) {
			// First check if we need to do the test through XML file, then ask the user
			if (choicesEntry.getValue().getTestMotor() && getUserConfirmation(choicesEntry.getKey())) {
				try {
					Method method = DiagnosticsOp.class.getDeclaredMethod(choicesEntry.getKey());

					choicesEntry.getValue().setErrorStatus((Boolean) method.invoke(this));
				} catch (NoSuchMethodException e) {
					Log.e(LOG_TAG, e.getMessage());
					choicesEntry.getValue().setErrorStatus(true);
					choicesEntry.getValue().addErrorMessage("Could not find method: " + choicesEntry.getKey());
				} catch (InvocationTargetException e) {
					Log.e(LOG_TAG, e.getMessage());
					choicesEntry.getValue().setErrorStatus(true);
					choicesEntry.getValue().addErrorMessage("Could invoke method: " + choicesEntry.getKey());
				} catch (IllegalAccessException e) {
					Log.e(LOG_TAG, e.getMessage());
					choicesEntry.getValue().setErrorStatus(true);
					choicesEntry.getValue().addErrorMessage("Could not access method: " + choicesEntry.getKey());
				}
			}
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

	private void initialize() throws InterruptedException {
		robotLibrary = new vv_Lib(this);
	}

	private HashMap<String, ChoiceRecord> getTests() {
		HashMap<String, ChoiceRecord> formattedChoices = new HashMap<>();
		HashMap<String, Boolean> xmlChoices;

		// Parse XML and get hashmap
		XmlParser xmlParser = new XmlParser();
		xmlChoices = xmlParser.getChoicesMap();

		for (String key : xmlChoices.keySet()) {
			// Take xml's hashmap and convert it into the one we want
			formattedChoices.put(key, new ChoiceRecord(xmlChoices.get(key)));
		}

		return formattedChoices;
	}
}
