/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
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
package org.recast4j.demo.tool;

import static org.lwjgl.nuklear.Nuklear.NK_TEXT_ALIGN_LEFT;
import static org.lwjgl.nuklear.Nuklear.nk_button_text;
import static org.lwjgl.nuklear.Nuklear.nk_label;
import static org.lwjgl.nuklear.Nuklear.nk_layout_row_dynamic;
import static org.lwjgl.nuklear.Nuklear.nk_option_label;
import static org.lwjgl.nuklear.Nuklear.nk_property_float;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.lwjgl.BufferUtils;
import org.lwjgl.nuklear.NkContext;
import org.recast4j.demo.builder.SampleAreaModifications;
import org.recast4j.demo.draw.DebugDraw;
import org.recast4j.demo.draw.DebugDrawPrimitives;
import org.recast4j.demo.draw.NavMeshRenderer;
import org.recast4j.demo.draw.RecastDebugDraw;
import org.recast4j.demo.geom.DemoInputGeomProvider;
import org.recast4j.demo.math.DemoMath;
import org.recast4j.demo.sample.Sample;
import org.recast4j.recast.AreaModification;
import org.recast4j.recast.ConvexVolume;

public class ConvexVolumeTool implements Tool {

    private Sample m_sample;
    private AreaModification m_areaType = SampleAreaModifications.SAMPLE_AREAMOD_GRASS;
    private final FloatBuffer m_boxHeight = BufferUtils.createFloatBuffer(1).put(0, 6f);
    private final FloatBuffer m_boxDescent = BufferUtils.createFloatBuffer(1).put(0, 1f);
    private final FloatBuffer m_polyOffset = BufferUtils.createFloatBuffer(1).put(0, 0f);
    private final List<Float> m_pts = new ArrayList<>();
    private final List<Integer> m_hull = new ArrayList<>();

    // Calculates convex hull on xz-plane of points on 'pts',
    // stores the indices of the resulting hull in 'out' and
    // returns number of points on hull.
    List<Integer> convexhull(List<Float> pts) {
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
    boolean cmppt(float[] a, float[] b) {
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
    boolean left(float[] a, float[] b, float[] c) {
        float u1 = b[0] - a[0];
        float v1 = b[2] - a[2];
        float u2 = c[0] - a[0];
        float v2 = c[2] - a[2];
        return u1 * v2 - v1 * u2 < 0;
    }

    boolean pointInPoly(float[] verts, float[] p) {
        int i, j;
        boolean c = false;
        for (i = 0, j = verts.length - 1; i < verts.length; j = i++) {
            float[] vi = new float[] { verts[i * 3], verts[i * 3 + 1], verts[i * 3 + 2] };
            float[] vj = new float[] { verts[j * 3], verts[j * 3 + 1], verts[j * 3 + 2] };
            if (((vi[2] > p[2]) != (vj[2] > p[2]))
                    && (p[0] < (vj[0] - vi[0]) * (p[2] - vi[2]) / (vj[2] - vi[2]) + vi[0])) {
                c = !c;
            }
        }
        return c;
    }

    @Override
    public void setSample(Sample m_sample) {
        this.m_sample = m_sample;
    }

    @Override
    public void handleClick(float[] s, float[] p, boolean shift) {
        // TODO Auto-generated method stub
        if (m_sample == null) {
            return;
        }
        DemoInputGeomProvider geom = m_sample.getInputGeom();
        if (geom == null) {
            return;
        }

        if (shift) {
            // Delete
            int nearestIndex = -1;
            List<ConvexVolume> vols = geom.convexVolumes();
            for (int i = 0; i < vols.size(); ++i) {
                if (pointInPoly(vols.get(i).verts, p) && p[1] >= vols.get(i).hmin && p[1] <= vols.get(i).hmax) {
                    nearestIndex = i;
                }
            }
            // If end point close enough, delete it.
            if (nearestIndex != -1) {
                geom.convexVolumes().remove(nearestIndex);
            }
        } else {
            // Create

            // If clicked on that last pt, create the shape.
            if (m_pts.size() > 0 && DemoMath.vDistSqr(p, new float[] { m_pts.get(m_pts.size() - 3),
                    m_pts.get(m_pts.size() - 2), m_pts.get(m_pts.size() - 1) }, 0) < 0.2f * 0.2f) {
                if (m_hull.size() > 2) {
                    // Create shape.
                    float[] verts = new float[m_hull.size() * 3];
                    for (int i = 0; i < m_hull.size(); ++i) {
                        verts[i * 3] = m_pts.get(m_hull.get(i) * 3);
                        verts[i * 3 + 1] = m_pts.get(m_hull.get(i) * 3 + 1);
                        verts[i * 3 + 2] = m_pts.get(m_hull.get(i) * 3 + 2);
                    }

                    float minh = Float.MAX_VALUE, maxh = 0;
                    for (int i = 0; i < m_hull.size(); ++i) {
                        minh = Math.min(minh, verts[i * 3 + 1]);
                    }
                    minh -= m_boxDescent.get(0);
                    maxh = minh + m_boxHeight.get(0);

                    if (m_polyOffset.get(0) > 0.01f) {
                        float[] offset = new float[verts.length * 2];
                        int noffset = offsetPoly(verts, m_hull.size(), m_polyOffset.get(0), offset, offset.length);
                        if (noffset > 0) {
                            geom.addConvexVolume(Arrays.copyOfRange(offset, 0, noffset * 3), minh, maxh, m_areaType);
                        }
                    } else {
                        geom.addConvexVolume(verts, minh, maxh, m_areaType);
                    }
                }
                m_pts.clear();
                m_hull.clear();
            } else {
                // Add new point
                m_pts.add(p[0]);
                m_pts.add(p[1]);
                m_pts.add(p[2]);
                // Update hull.
                if (m_pts.size() > 3) {
                    m_hull.clear();
                    m_hull.addAll(convexhull(m_pts));
                } else {
                    m_hull.clear();
                }
            }
        }

    }

    int offsetPoly(float[] verts, int nverts, float offset, float[] outVerts, int maxOutVerts) {
        float MITER_LIMIT = 1.20f;

        int n = 0;

        for (int i = 0; i < nverts; i++) {
            int a = (i + nverts - 1) % nverts;
            int b = i;
            int c = (i + 1) % nverts;
            int va = a * 3;
            int vb = b * 3;
            int vc = c * 3;
            float dx0 = verts[vb] - verts[va];
            float dy0 = verts[vb + 2] - verts[va + 2];
            float d0 = dx0 * dx0 + dy0 * dy0;
            if (d0 > 1e-6f) {
                d0 = (float) (1.0f / Math.sqrt(d0));
                dx0 *= d0;
                dy0 *= d0;
            }
            float dx1 = verts[vc] - verts[vb];
            float dy1 = verts[vc + 2] - verts[vb + 2];
            float d1 = dx1 * dx1 + dy1 * dy1;
            if (d1 > 1e-6f) {
                d1 = (float) (1.0f / Math.sqrt(d1));
                dx1 *= d1;
                dy1 *= d1;
            }
            float dlx0 = -dy0;
            float dly0 = dx0;
            float dlx1 = -dy1;
            float dly1 = dx1;
            float cross = dx1 * dy0 - dx0 * dy1;
            float dmx = (dlx0 + dlx1) * 0.5f;
            float dmy = (dly0 + dly1) * 0.5f;
            float dmr2 = dmx * dmx + dmy * dmy;
            boolean bevel = dmr2 * MITER_LIMIT * MITER_LIMIT < 1.0f;
            if (dmr2 > 1e-6f) {
                float scale = 1.0f / dmr2;
                dmx *= scale;
                dmy *= scale;
            }

            if (bevel && cross < 0.0f) {
                if (n + 2 >= maxOutVerts) {
                    return 0;
                }
                float d = (1.0f - (dx0 * dx1 + dy0 * dy1)) * 0.5f;
                outVerts[n * 3 + 0] = verts[vb] + (-dlx0 + dx0 * d) * offset;
                outVerts[n * 3 + 1] = verts[vb + 1];
                outVerts[n * 3 + 2] = verts[vb + 2] + (-dly0 + dy0 * d) * offset;
                n++;
                outVerts[n * 3 + 0] = verts[vb] + (-dlx1 - dx1 * d) * offset;
                outVerts[n * 3 + 1] = verts[vb + 1];
                outVerts[n * 3 + 2] = verts[vb + 2] + (-dly1 - dy1 * d) * offset;
                n++;
            } else {
                if (n + 1 >= maxOutVerts) {
                    return 0;
                }
                outVerts[n * 3 + 0] = verts[vb] - dmx * offset;
                outVerts[n * 3 + 1] = verts[vb + 1];
                outVerts[n * 3 + 2] = verts[vb + 2] - dmy * offset;
                n++;
            }
        }

        return n;
    }

    @Override
    public void handleRender(NavMeshRenderer renderer) {
        RecastDebugDraw dd = renderer.getDebugDraw();
        // Find height extent of the shape.
        float minh = Float.MAX_VALUE, maxh = 0;
        for (int i = 0; i < m_pts.size(); i += 3) {
            minh = Math.min(minh, m_pts.get(i + 1));
        }
        minh -= m_boxDescent.get(0);
        maxh = minh + m_boxHeight.get(0);

        dd.begin(DebugDrawPrimitives.POINTS, 4.0f);
        for (int i = 0; i < m_pts.size(); i += 3) {
            int col = DebugDraw.duRGBA(255, 255, 255, 255);
            if (i == m_pts.size() - 3) {
                col = DebugDraw.duRGBA(240, 32, 16, 255);
            }
            dd.vertex(m_pts.get(i + 0), m_pts.get(i + 1) + 0.1f, m_pts.get(i + 2), col);
        }
        dd.end();

        dd.begin(DebugDrawPrimitives.LINES, 2.0f);
        for (int i = 0, j = m_hull.size() - 1; i < m_hull.size(); j = i++) {
            int vi = m_hull.get(j) * 3;
            int vj = m_hull.get(i) * 3;
            dd.vertex(m_pts.get(vj + 0), minh, m_pts.get(vj + 2), DebugDraw.duRGBA(255, 255, 255, 64));
            dd.vertex(m_pts.get(vi + 0), minh, m_pts.get(vi + 2), DebugDraw.duRGBA(255, 255, 255, 64));
            dd.vertex(m_pts.get(vj + 0), maxh, m_pts.get(vj + 2), DebugDraw.duRGBA(255, 255, 255, 64));
            dd.vertex(m_pts.get(vi + 0), maxh, m_pts.get(vi + 2), DebugDraw.duRGBA(255, 255, 255, 64));
            dd.vertex(m_pts.get(vj + 0), minh, m_pts.get(vj + 2), DebugDraw.duRGBA(255, 255, 255, 64));
            dd.vertex(m_pts.get(vj + 0), maxh, m_pts.get(vj + 2), DebugDraw.duRGBA(255, 255, 255, 64));
        }
        dd.end();
    }

    @Override
    public void layout(NkContext ctx) {
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Shape Height", 0.1f, m_boxHeight, 20f, 0.1f, 0.1f);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Shape Descent", 0.1f, m_boxDescent, 20f, 0.1f, 0.1f);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Poly Offset", 0.1f, m_polyOffset, 10f, 0.1f, 0.1f);
        nk_label(ctx, "Area Type", NK_TEXT_ALIGN_LEFT);
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Ground", m_areaType == SampleAreaModifications.SAMPLE_AREAMOD_GROUND)) {
            m_areaType = SampleAreaModifications.SAMPLE_AREAMOD_GROUND;
        }
        if (nk_option_label(ctx, "Water", m_areaType == SampleAreaModifications.SAMPLE_AREAMOD_WATER)) {
            m_areaType = SampleAreaModifications.SAMPLE_AREAMOD_WATER;
        }
        if (nk_option_label(ctx, "Road", m_areaType == SampleAreaModifications.SAMPLE_AREAMOD_ROAD)) {
            m_areaType = SampleAreaModifications.SAMPLE_AREAMOD_ROAD;
        }
        if (nk_option_label(ctx, "Door", m_areaType == SampleAreaModifications.SAMPLE_AREAMOD_DOOR)) {
            m_areaType = SampleAreaModifications.SAMPLE_AREAMOD_DOOR;
        }
        if (nk_option_label(ctx, "Grass", m_areaType == SampleAreaModifications.SAMPLE_AREAMOD_GRASS)) {
            m_areaType = SampleAreaModifications.SAMPLE_AREAMOD_GRASS;
        }
        if (nk_option_label(ctx, "Jump", m_areaType == SampleAreaModifications.SAMPLE_AREAMOD_JUMP)) {
            m_areaType = SampleAreaModifications.SAMPLE_AREAMOD_JUMP;
        }
        if (nk_button_text(ctx, "Clear Shape")) {
            m_hull.clear();
            m_pts.clear();
        }
    }

    @Override
    public String getName() {
        return "Create Convex Volumes";
    }

    @Override
    public void handleUpdate(float dt) {
        // TODO Auto-generated method stub
    }

    public static void main(String[] args) {
        ConvexVolumeTool t = new ConvexVolumeTool();
        List<Float> points = new ArrayList<>();
        //  0
        points.add(0f);
        points.add(-10f);
        points.add(3f);
        //  1
        points.add(1f);
        points.add(-10f);
        points.add(1f);
        //  2
        points.add(2f);
        points.add(-10f);
        points.add(2f);
        //  3
        points.add(4f);
        points.add(-10f);
        points.add(4f);
        //  4
        points.add(0f);
        points.add(-10f);
        points.add(0f);
        //  5
        points.add(1f);
        points.add(-10f);
        points.add(2f);
        //  6
        points.add(3f);
        points.add(-10f);
        points.add(1f);
        //  7
        points.add(3f);
        points.add(-10f);
        points.add(3f);
        List<Integer> h = t.convexhull(points);
        System.err.println(h);
    }
}
