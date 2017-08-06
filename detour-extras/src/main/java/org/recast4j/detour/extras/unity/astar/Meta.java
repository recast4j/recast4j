package org.recast4j.detour.extras.unity.astar;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

class Meta {

	static String TYPENAME_RECAST_GRAPH = "Pathfinding.RecastGraph";
	static String MIN_SUPPORTED_VERSION = "4.0.6";
	static Pattern VERSION_PATTERN = Pattern.compile("(\\d+)\\.(\\d+)\\.(\\d+)");
	String version;
	int graphs;
	String[] typeNames;

	boolean isSupportedVersion() {
		int[] actual = parseVersion(version);
		int[] minSupported = parseVersion(MIN_SUPPORTED_VERSION);
		for (int i = 0; i < Math.min(actual.length, minSupported.length); i++) {
			if (actual[i] > minSupported[i]) {
				return true;
			} else if (minSupported[i] > actual[i]) {
				return false;
			}
		}
		return true;
	}
	
	private int[] parseVersion(String version) {
		Matcher m = VERSION_PATTERN.matcher(version);
		if (m.matches()) {
			int[] v = new int[m.groupCount()];
			for (int i = 0; i < v.length; i++) {
				v[i] = Integer.parseInt(m.group(i + 1));
			}
			return v;
		}
		throw new IllegalArgumentException("Invalid version format: " + version);
	}

	boolean isSupportedType() {
		for (String t : typeNames) {
			if (t.equals(TYPENAME_RECAST_GRAPH)) {
				return true;
			}
		}
		return false;
	}

}
