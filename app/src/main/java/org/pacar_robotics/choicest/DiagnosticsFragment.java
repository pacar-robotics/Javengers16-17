package org.pacar_robotics.choicest;


import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import org.pacar_robotics.choicest.R;

/**
 * A simple {@link Fragment} subclass.
 */
public class DiagnosticsFragment extends Fragment {


	public DiagnosticsFragment() {
		// Required empty public constructor
	}


	@Override
	public View onCreateView(LayoutInflater inflater, ViewGroup container,
	                         Bundle savedInstanceState) {
		// Inflate the layout for this fragment
		return inflater.inflate(R.layout.fragment_diagnostics, container, false);
	}

}
