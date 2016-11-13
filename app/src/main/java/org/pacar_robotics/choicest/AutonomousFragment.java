package org.pacar_robotics.choicest;


import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.app.Fragment;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CheckBox;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import org.w3c.dom.Document;
import org.w3c.dom.Element;

import java.io.File;
import java.util.LinkedHashMap;
import java.util.Map;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.OnCheckedChanged;
import butterknife.OnClick;
import butterknife.Unbinder;


/**
 * A simple {@link Fragment} subclass.
 */
public class AutonomousFragment extends Fragment {

	private static final int MY_PERMISSION_REQUEST_STORAGE = 101;
	private static final String LOG_TAG = "MainActivity";
	private static final String FILE = Environment.getExternalStorageDirectory().getPath() +
			"/PACAR/AutoChoices.xml";

	@BindView(R.id.seek_delay) SeekBar delaySeek;
	@BindView(R.id.text_delay) TextView delayText;
	@BindView(R.id.radio_group_alliance) RadioGroup allianceGroup;
	@BindView(R.id.radio_group_starting_position) RadioGroup startingPositionGroup;
	@BindView(R.id.beacon_1) CheckBox beacon1Check;
	@BindView(R.id.beacon_2) CheckBox beacon2Check;
	@BindView(R.id.center_vortex) CheckBox centerVortexCheck;
	@BindView(R.id.corner_vortex) CheckBox cornerVortexCheck;
	@BindView(R.id.block) CheckBox blockCheck;

	private Unbinder unbinder;

	private static final String ARG_SECTION_NUMBER = "section_number";

	public AutonomousFragment() {
		// Required empty public constructor
	}

	@Override
	public View onCreateView(LayoutInflater inflater, ViewGroup container,
	                         Bundle savedInstanceState) {
		View rootView = inflater.inflate(R.layout.fragment_autonomous, container, false);

		unbinder = ButterKnife.bind(this, rootView);

		delaySeek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
				// Update the delay text view to reflect current choice
				delayText.setText(Integer.toString(progress));
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
			}
		});

		return rootView;
	}

	@OnClick(R.id.btn_save)
	void saveButtonClicked() {
		if ((Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) &&
				(ContextCompat.checkSelfPermission(getActivity(), Manifest.permission.WRITE_EXTERNAL_STORAGE)
						!= PackageManager.PERMISSION_GRANTED)) {
			// If system OS is Marshmallow or greater, must check if have dangerous storage permission
			ActivityCompat.requestPermissions(getActivity(), new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE},
					MY_PERMISSION_REQUEST_STORAGE);
		} else {
			// Permission is granted, move on
			Log.v(LOG_TAG, "External Storage Permission granted");
			writeXML(createHashMap());
			Log.v(LOG_TAG, "Finished writeXML function");
		}
	}

	@OnCheckedChanged(R.id.center_vortex)
	void centerVortexChecked() {
		if (centerVortexCheck.isChecked()) {
			cornerVortexCheck.setChecked(false);
		}
	}

	@OnCheckedChanged(R.id.corner_vortex)
	void cornerVortexChecked() {
		if (cornerVortexCheck.isChecked()) {
			centerVortexCheck.setChecked(false);
		}
	}

	private LinkedHashMap<String, String> createHashMap() {
		LinkedHashMap<String, String> choicesMap = new LinkedHashMap<>();

		// We have to remove spaces between the tags or else it will crash
		// Alliance
		RadioButton allianceButton = (RadioButton) getView().findViewById(allianceGroup.getCheckedRadioButtonId());
		choicesMap.put(getString(R.string.alliance_header).replace(" ", ""), allianceButton.getText().toString());

		// Starting position
		RadioButton startingPositionButton = (RadioButton) getView().findViewById(startingPositionGroup.getCheckedRadioButtonId());
		choicesMap.put(getString(R.string.starting_position_header).replace(" ", ""), startingPositionButton.getText().toString());

		// Checkbox Choices
		choicesMap.put(getString(R.string.beacon_1).replace(" ", ""), Boolean.toString(beacon1Check.isChecked()));
		choicesMap.put(getString(R.string.beacon_2).replace(" ", ""), Boolean.toString(beacon2Check.isChecked()));
		choicesMap.put(getString(R.string.park_on_center_vortex_partially).replace(" ", ""), Boolean.toString(centerVortexCheck.isChecked()));
		choicesMap.put(getString(R.string.park_on_corner_vortex).replace(" ", ""), Boolean.toString(cornerVortexCheck.isChecked()));
		choicesMap.put(getString(R.string.block_opposing_team).replace(" ", ""), Boolean.toString(blockCheck.isChecked()));

		// Delay
		choicesMap.put(getString(R.string.delay_header).replace(" ", ""), Integer.toString(delaySeek.getProgress()));

		return choicesMap;
	}

	@Override
	public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
	                                       @NonNull int[] grantResults) {
		super.onRequestPermissionsResult(requestCode, permissions, grantResults);

		switch (requestCode) {
			case MY_PERMISSION_REQUEST_STORAGE:
				if (grantResults[0] == PackageManager.PERMISSION_GRANTED) {
					// Permission has been granted
					// Go back into saveButtonClicked because it will go into else and save the file
					saveButtonClicked();
				} else {
					if (ActivityCompat.shouldShowRequestPermissionRationale(getActivity(),
							Manifest.permission.WRITE_EXTERNAL_STORAGE)) {
						// Permission got denied but user did not check "do not show again"
						// They may have clicked the wrong button, so tell them they clicked the wrong one
						Toast.makeText(getActivity(), "We need the storage permission to make an xml file\n" +
								"Please grant this permission", Toast.LENGTH_LONG).show();
					} else {
						// If they clicked "Do not show again", the user is not smart enough to be on this team
						Toast.makeText(getActivity(), "You don't know how to use this app.\n" +
										"Ask Rahul the Tech Support God to help you",
								Toast.LENGTH_LONG).show();
					}
				}
				break;

			default:
				// There is no way the program can get to this statement
				// If it does, something is very, very wrong
				Log.e(LOG_TAG, "Reached default statement of onRequestPermissionResult");
				Toast.makeText(getActivity(), "Something bad happened", Toast.LENGTH_SHORT).show();
				break;
		}
	}

	private void writeXML(LinkedHashMap<String, String> choicesMap) {
		File file = new File(FILE);

		if (file.delete()) {
			Log.v(LOG_TAG, " File deleted:" + FILE);
		} else {
			Log.v(LOG_TAG, " File NOT deleted:" + FILE);
		}

		try {
			DocumentBuilderFactory docFactory = DocumentBuilderFactory.newInstance();
			DocumentBuilder docBuilder = docFactory.newDocumentBuilder();

			// Create root elements
			Document doc = docBuilder.newDocument();
			Element root = doc.createElement("AutoChoices");
			doc.appendChild(root);

			for (Map.Entry<String, String> entry : choicesMap.entrySet()) {
				Element i = doc.createElement(entry.getKey());
				i.appendChild(doc.createTextNode(entry.getValue()));
				root.appendChild(i);
			}

			// Write the content into xml file
			TransformerFactory transformerFactory = TransformerFactory.newInstance();
			Transformer transformer = transformerFactory.newTransformer();

			DOMSource source = new DOMSource(doc);

			StreamResult result = new StreamResult(file);

			// Fix XML formatting
			transformer.setOutputProperty(OutputKeys.INDENT, "yes");
			transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");

			transformer.transform(source, result);
		} catch (Exception e) {
			Toast.makeText(getActivity(), "Oh no! File creation failed!", Toast.LENGTH_LONG).show();
			Log.e(LOG_TAG, "File creation failed: " + e.getMessage());
		}
	}

	@Override
	public void onDestroyView() {
		super.onDestroyView();
		unbinder.unbind();
	}
}
