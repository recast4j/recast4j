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
import static org.recast4j.demo.draw.DebugDraw.duRGBA;
import static org.recast4j.demo.draw.DebugDrawPrimitives.LINES;
import static org.recast4j.demo.draw.DebugDrawPrimitives.POINTS;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.lwjgl.BufferUtils;
import org.lwjgl.nuklear.NkContext;
import org.recast4j.demo.builder.SampleAreaModifications;
import org.recast4j.demo.draw.NavMeshRenderer;
import org.recast4j.demo.draw.RecastDebugDraw;
import org.recast4j.demo.geom.DemoInputGeomProvider;
import org.recast4j.demo.math.ConvexUtils;
import org.recast4j.demo.math.DemoMath;
import org.recast4j.demo.sample.Sample;
import org.recast4j.recast.AreaModification;
import org.recast4j.recast.ConvexVolume;

public class ConvexVolumeTool implements Tool {

    private Sample sample;
    private AreaModification areaType = SampleAreaModifications.SAMPLE_AREAMOD_GRASS;
    private final FloatBuffer boxHeight = BufferUtils.createFloatBuffer(1).put(0, 6f);
    private final FloatBuffer boxDescent = BufferUtils.createFloatBuffer(1).put(0, 1f);
    private final FloatBuffer polyOffset = BufferUtils.createFloatBuffer(1).put(0, 0f);
    private final List<Float> pts = new ArrayList<>();
    private final List<Integer> hull = new ArrayList<>();

    @Override
    public void setSample(Sample m_sample) {
        sample = m_sample;
    }

    @Override
    public void handleClick(float[] s, float[] p, boolean shift) {
        // TODO Auto-generated method stub
        if (sample == null) {
            return;
        }
        DemoInputGeomProvider geom = sample.getInputGeom();
        if (geom == null) {
            return;
        }

        if (shift) {
            // Delete
            int nearestIndex = -1;
            List<ConvexVolume> vols = geom.convexVolumes();
            for (int i = 0; i < vols.size(); ++i) {
                if (PolyUtils.pointInPoly(vols.get(i).verts, p) && p[1] >= vols.get(i).hmin
                        && p[1] <= vols.get(i).hmax) {
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
            if (pts.size() > 0 && DemoMath.vDistSqr(p,
                    new float[] { pts.get(pts.size() - 3), pts.get(pts.size() - 2), pts.get(pts.size() - 1) },
                    0) < 0.2f * 0.2f) {
                if (hull.size() > 2) {
                    // Create shape.
                    float[] verts = new float[hull.size() * 3];
                    for (int i = 0; i < hull.size(); ++i) {
                        verts[i * 3] = pts.get(hull.get(i) * 3);
                        verts[i * 3 + 1] = pts.get(hull.get(i) * 3 + 1);
                        verts[i * 3 + 2] = pts.get(hull.get(i) * 3 + 2);
                    }

                    float minh = Float.MAX_VALUE, maxh = 0;
                    for (int i = 0; i < hull.size(); ++i) {
                        minh = Math.min(minh, verts[i * 3 + 1]);
                    }
                    minh -= boxDescent.get(0);
                    maxh = minh + boxHeight.get(0);

                    if (polyOffset.get(0) > 0.01f) {
                        float[] offset = new float[verts.length * 2];
                        int noffset = PolyUtils.offsetPoly(verts, hull.size(), polyOffset.get(0), offset,
                                offset.length);
                        if (noffset > 0) {
                            geom.addConvexVolume(Arrays.copyOfRange(offset, 0, noffset * 3), minh, maxh, areaType);
                        }
                    } else {
                        geom.addConvexVolume(verts, minh, maxh, areaType);
                    }
                }
                pts.clear();
                hull.clear();
            } else {
                // Add new point
                pts.add(p[0]);
                pts.add(p[1]);
                pts.add(p[2]);
                // Update hull.
                if (pts.size() > 3) {
                    hull.clear();
                    hull.addAll(ConvexUtils.convexhull(pts));
                } else {
                    hull.clear();
                }
            }
        }

    }

    @Override
    public void handleRender(NavMeshRenderer renderer) {
        RecastDebugDraw dd = renderer.getDebugDraw();
        // Find height extent of the shape.
        float minh = Float.MAX_VALUE, maxh = 0;
        for (int i = 0; i < pts.size(); i += 3) {
            minh = Math.min(minh, pts.get(i + 1));
        }
        minh -= boxDescent.get(0);
        maxh = minh + boxHeight.get(0);

        dd.begin(POINTS, 4.0f);
        for (int i = 0; i < pts.size(); i += 3) {
            int col = duRGBA(255, 255, 255, 255);
            if (i == pts.size() - 3) {
                col = duRGBA(240, 32, 16, 255);
            }
            dd.vertex(pts.get(i + 0), pts.get(i + 1) + 0.1f, pts.get(i + 2), col);
        }
        dd.end();

        dd.begin(LINES, 2.0f);
        for (int i = 0, j = hull.size() - 1; i < hull.size(); j = i++) {
            int vi = hull.get(j) * 3;
            int vj = hull.get(i) * 3;
            dd.vertex(pts.get(vj + 0), minh, pts.get(vj + 2), duRGBA(255, 255, 255, 64));
            dd.vertex(pts.get(vi + 0), minh, pts.get(vi + 2), duRGBA(255, 255, 255, 64));
            dd.vertex(pts.get(vj + 0), maxh, pts.get(vj + 2), duRGBA(255, 255, 255, 64));
            dd.vertex(pts.get(vi + 0), maxh, pts.get(vi + 2), duRGBA(255, 255, 255, 64));
            dd.vertex(pts.get(vj + 0), minh, pts.get(vj + 2), duRGBA(255, 255, 255, 64));
            dd.vertex(pts.get(vj + 0), maxh, pts.get(vj + 2), duRGBA(255, 255, 255, 64));
        }
        dd.end();
    }

    @Override
    public void layout(NkContext ctx) {
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Shape Height", 0.1f, boxHeight, 20f, 0.1f, 0.1f);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Shape Descent", 0.1f, boxDescent, 20f, 0.1f, 0.1f);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Poly Offset", 0.1f, polyOffset, 10f, 0.1f, 0.1f);
        nk_label(ctx, "Area Type", NK_TEXT_ALIGN_LEFT);
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Ground", areaType == SampleAreaModifications.SAMPLE_AREAMOD_GROUND)) {
            areaType = SampleAreaModifications.SAMPLE_AREAMOD_GROUND;
        }
        if (nk_option_label(ctx, "Water", areaType == SampleAreaModifications.SAMPLE_AREAMOD_WATER)) {
            areaType = SampleAreaModifications.SAMPLE_AREAMOD_WATER;
        }
        if (nk_option_label(ctx, "Road", areaType == SampleAreaModifications.SAMPLE_AREAMOD_ROAD)) {
            areaType = SampleAreaModifications.SAMPLE_AREAMOD_ROAD;
        }
        if (nk_option_label(ctx, "Door", areaType == SampleAreaModifications.SAMPLE_AREAMOD_DOOR)) {
            areaType = SampleAreaModifications.SAMPLE_AREAMOD_DOOR;
        }
        if (nk_option_label(ctx, "Grass", areaType == SampleAreaModifications.SAMPLE_AREAMOD_GRASS)) {
            areaType = SampleAreaModifications.SAMPLE_AREAMOD_GRASS;
        }
        if (nk_option_label(ctx, "Jump", areaType == SampleAreaModifications.SAMPLE_AREAMOD_JUMP)) {
            areaType = SampleAreaModifications.SAMPLE_AREAMOD_JUMP;
        }
        if (nk_button_text(ctx, "Clear Shape")) {
            hull.clear();
            pts.clear();
        }
        if (nk_button_text(ctx, "Remove All")) {
            hull.clear();
            pts.clear();
            DemoInputGeomProvider geom = sample.getInputGeom();
            if (geom != null) {
                geom.clearConvexVolumes();
            }
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

}
