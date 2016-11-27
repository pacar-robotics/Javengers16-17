package org.pacar_robotics.choicest;


import android.os.Bundle;
import android.os.Environment;
import android.support.annotation.NonNull;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CheckBox;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.TextView;

import java.util.HashMap;
import java.util.List;

import butterknife.BindView;
import butterknife.BindViews;
import butterknife.ButterKnife;
import butterknife.OnCheckedChanged;
import butterknife.OnClick;
import butterknife.Unbinder;


/**
 * A simple {@link Fragment} subclass.
 */
public class AutonomousFragment extends Fragment {

	private static final int MY_PERMISSION_REQUEST_STORAGE = 101;
	private static final String LOG_TAG = "AutonomousFragment";
	private static final String FILE = Environment.getExternalStorageDirectory().getPath() +
			"/PACAR/AutoChoices.xml";

	@BindView(R.id.seek_delay) SeekBar delaySeek;
	@BindView(R.id.text_delay) TextView delayText;
	@BindView(R.id.radio_group_alliance) RadioGroup allianceGroup;
	@BindView(R.id.radio_group_starting_position) RadioGroup startingPositionGroup;
	@BindView(R.id.center_vortex) CheckBox centerVortexCheck;
	@BindView(R.id.corner_vortex) CheckBox cornerVortexCheck;

	@BindViews({R.id.beacon_1, R.id.beacon_2, R.id.center_vortex, R.id.corner_vortex, R.id.block})
	List<CheckBox> choicesList;

	private Unbinder unbinder;

	XmlWriter xmlWriter;

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
		xmlWriter = new XmlWriter(FILE, createHashMap(), getActivity());
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

	private HashMap<String, String> createHashMap() {
		HashMap<String, String> choicesMap = new HashMap<>();

		// We have to remove spaces between the tags or else it will crash
		// Alliance
		RadioButton allianceButton = (RadioButton) getView().findViewById(allianceGroup.getCheckedRadioButtonId());
		choicesMap.put(getString(R.string.alliance_header).replace(" ", ""), allianceButton.getText().toString());

		// Starting position
		RadioButton startingPositionButton = (RadioButton) getView().findViewById(startingPositionGroup.getCheckedRadioButtonId());
		choicesMap.put(getString(R.string.starting_position_header).replace(" ", ""), startingPositionButton.getText().toString());

		for (CheckBox checkBox : choicesList) {
			choicesMap.put(checkBox.getText().toString().replace(" ", "").toLowerCase(),
					Boolean.toString(checkBox.isChecked()));
		}

		// Delay
		choicesMap.put(getString(R.string.delay_header).replace(" ", ""), Integer.toString(delaySeek.getProgress()));

		return choicesMap;
	}

	@Override
	public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
	                                       @NonNull int[] grantResults) {
		super.onRequestPermissionsResult(requestCode, permissions, grantResults);

		xmlWriter.onRequestPermissionResult(requestCode, getActivity(), grantResults);
	}

	@Override
	public void onDestroyView() {
		super.onDestroyView();
		unbinder.unbind();
	}
}
