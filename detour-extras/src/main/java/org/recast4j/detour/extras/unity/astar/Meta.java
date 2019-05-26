/*
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
package org.recast4j.detour.extras.unity.astar;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

class Meta {

    final static String TYPENAME_RECAST_GRAPH = "Pathfinding.RecastGraph";
    final static String MIN_SUPPORTED_VERSION = "4.0.6";
    final static String UPDATED_STRUCT_VERSION = "4.1.16";
    final static Pattern VERSION_PATTERN = Pattern.compile("(\\d+)\\.(\\d+)\\.(\\d+)");
    String version;
    int graphs;
    String[] typeNames;

    boolean isSupportedVersion() {
        return isSupportedVersion(MIN_SUPPORTED_VERSION);
    }

    boolean isSupportedVersion(String minVersion) {
        int[] actual = parseVersion(version);
        int[] minSupported = parseVersion(minVersion);
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
