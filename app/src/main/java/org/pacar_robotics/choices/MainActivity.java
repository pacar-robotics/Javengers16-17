package org.pacar_robotics.choices;

import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.OnClick;

public class MainActivity extends AppCompatActivity {

	private static final int MY_PERMISSION_REQUEST_STORAGE = 101;

	@BindView(R.id.seek_delay) SeekBar delaySeek;
	@BindView(R.id.text_delay) TextView delayText;

	@OnClick(R.id.btn_save)
	void saveButtonClicked() {
		if ((Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) &&
				(ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE)
						!= PackageManager.PERMISSION_GRANTED)) {
			// If system OS is Marshmallow or greater, must check if have dangerous storage permission
			ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE},
					MY_PERMISSION_REQUEST_STORAGE);
		} else {
			// Permission is granted, move on
			Toast.makeText(this, "Permission has been granted", Toast.LENGTH_SHORT).show();
			//TODO: create and save file
		}
	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		ButterKnife.bind(this);

		delaySeek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
				// Update the delay text view to reflect current choice
				delayText.setText(Integer.toString(progress));
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {}
		});
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
					if (ActivityCompat.shouldShowRequestPermissionRationale(this,
							Manifest.permission.WRITE_EXTERNAL_STORAGE)) {
						// Permission got denied but user did not check "do not show again"
						// They may have clicked the wrong button, so tell them they clicked the wrong one
						Toast.makeText(this, "We need the storage permission to make an xml file\n" +
								"Please grant this permission", Toast.LENGTH_LONG).show();
					} else {
						// If they clicked "Do not show again", the user is not smart enough to be on this team
						Toast.makeText(this, "You don't know how to use this app.\n" +
										"Ask Rahul the Tech Support God to help you",
								Toast.LENGTH_LONG).show();
					}
				}
				break;

			default:
				// There is no way the program can get to this statement
				// If it does, something is very, very wrong
				Toast.makeText(this, "Something bad happened", Toast.LENGTH_SHORT).show();
				break;
		}
	}
}