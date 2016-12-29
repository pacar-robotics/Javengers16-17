package org.pacar_robotics.choicest;

import android.util.Log;

import org.w3c.dom.Document;
import org.w3c.dom.NodeList;

import java.io.File;
import java.util.LinkedHashMap;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpression;
import javax.xml.xpath.XPathFactory;

/**
 * Created by Rahul on 12/29/2016.
 */

public class XmlParser {
	private String fileName;
	private static final String LOG_TAG = "XmlParser";

	private LinkedHashMap<String, Boolean> choicesMap;

	public XmlParser(String fileName) {
		this.fileName = fileName;
		choicesMap = new LinkedHashMap<>();
		parseXml();
	}

	private void parseXml() {
		try {
			File xmlFile = new File(fileName);
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
