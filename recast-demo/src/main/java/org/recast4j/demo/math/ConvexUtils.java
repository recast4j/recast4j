/*
recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.demo.math;

import java.util.ArrayList;
import java.util.List;

public class ConvexUtils {

    // Calculates convex hull on xz-plane of points on 'pts',
    // stores the indices of the resulting hull in 'out' and
    // returns number of points on hull.
    public static List<Integer> convexhull(List<Float> pts) {
        int npts = pts.size() / 3;
        List<Integer> out = new ArrayList<>();
        // Find lower-leftmost point.
        int hull = 0;
        for (int i = 1; i < npts; ++i) {
            float[] a = new float[] { pts.get(i * 3), pts.get(i * 3 + 1), pts.get(i * 3 + 2) };
            float[] b = new float[] { pts.get(hull * 3), pts.get(hull * 3 + 1), pts.get(hull * 3 + 2) };
            if (cmppt(a, b)) {
                hull = i;
            }
        }
        // Gift wrap hull.
        int endpt = 0;
        do {
            out.add(hull);
            endpt = 0;
            for (int j = 1; j < npts; ++j) {
                float[] a = new float[] { pts.get(hull * 3), pts.get(hull * 3 + 1), pts.get(hull * 3 + 2) };
                float[] b = new float[] { pts.get(endpt * 3), pts.get(endpt * 3 + 1), pts.get(endpt * 3 + 2) };
                float[] c = new float[] { pts.get(j * 3), pts.get(j * 3 + 1), pts.get(j * 3 + 2) };
                if (hull == endpt || left(a, b, c)) {
                    endpt = j;
                }
            }
            hull = endpt;
        } while (endpt != out.get(0));

        return out;
    }

    // Returns true if 'a' is more lower-left than 'b'.
    private static boolean cmppt(float[] a, float[] b) {
        if (a[0] < b[0]) {
            return true;
        }
        if (a[0] > b[0]) {
            return false;
        }
        if (a[2] < b[2]) {
            return true;
        }
        if (a[2] > b[2]) {
            return false;
        }
        return false;
    }

    // Returns true if 'c' is left of line 'a'-'b'.
    private static boolean left(float[] a, float[] b, float[] c) {
        float u1 = b[0] - a[0];
        float v1 = b[2] - a[2];
        float u2 = c[0] - a[0];
        float v2 = c[2] - a[2];
        return u1 * v2 - v1 * u2 < 0;
    }

}
