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

import static org.lwjgl.nuklear.Nuklear.nk_layout_row_dynamic;
import static org.lwjgl.nuklear.Nuklear.nk_option_label;
import static org.recast4j.demo.draw.DebugDraw.duRGBA;

import java.util.Arrays;

import org.lwjgl.nuklear.NkContext;
import org.recast4j.demo.builder.SampleAreaModifications;
import org.recast4j.demo.draw.NavMeshRenderer;
import org.recast4j.demo.draw.RecastDebugDraw;
import org.recast4j.demo.geom.DemoInputGeomProvider;
import org.recast4j.demo.geom.DemoOffMeshConnection;
import org.recast4j.demo.math.DemoMath;
import org.recast4j.demo.sample.Sample;

public class OffMeshConnectionTool implements Tool {

    private Sample sample;
    private boolean hitPosSet;
    private float[] hitPos;
    private boolean bidir;

    @Override
    public void setSample(Sample m_sample) {
        sample = m_sample;
    }

    @Override
    public void handleClick(float[] s, float[] p, boolean shift) {
        if (sample == null) {
            return;
        }
        DemoInputGeomProvider geom = sample.getInputGeom();
        if (geom == null) {
            return;
        }

        if (shift) {
            // Delete
            // Find nearest link end-point
            float nearestDist = Float.MAX_VALUE;
            DemoOffMeshConnection nearestConnection = null;
            for (DemoOffMeshConnection offMeshCon : geom.getOffMeshConnections()) {
                float d = Math.min(DemoMath.vDistSqr(p, offMeshCon.verts, 0), DemoMath.vDistSqr(p, offMeshCon.verts, 3));
                if (d < nearestDist && Math.sqrt(d) < sample.getSettingsUI().getAgentRadius()) {
                    nearestDist = d;
                    nearestConnection = offMeshCon;
                }
            }
            if (nearestConnection != null) {
                geom.getOffMeshConnections().remove(nearestConnection);
            }
        } else {
            // Create
            if (!hitPosSet) {
                hitPos = Arrays.copyOf(p, p.length);
                hitPosSet = true;
            } else {
                int area = SampleAreaModifications.SAMPLE_POLYAREA_TYPE_JUMP;
                int flags = SampleAreaModifications.SAMPLE_POLYFLAGS_JUMP;
                geom.addOffMeshConnection(hitPos, p, sample.getSettingsUI().getAgentRadius(), bidir, area, flags);
                hitPosSet = false;
            }
        }

    }

    @Override
    public void handleRender(NavMeshRenderer renderer) {
        if (sample == null) {
            return;
        }
        RecastDebugDraw dd = renderer.getDebugDraw();
        float s = sample.getSettingsUI().getAgentRadius();

        if (hitPosSet) {
            dd.debugDrawCross(hitPos[0], hitPos[1] + 0.1f, hitPos[2], s, duRGBA(0, 0, 0, 128), 2.0f);
        }
        DemoInputGeomProvider geom = sample.getInputGeom();
        if (geom != null) {
            renderer.drawOffMeshConnections(geom, true);
        }
    }

    @Override
    public void layout(NkContext ctx) {
        nk_layout_row_dynamic(ctx, 20, 1);
        bidir = !nk_option_label(ctx, "One Way", !bidir);
        nk_layout_row_dynamic(ctx, 20, 1);
        bidir = nk_option_label(ctx, "Bidirectional", bidir);
    }

    @Override
    public String getName() {
        return "Create Off-Mesh Links";
    }

    @Override
    public void handleUpdate(float dt) {
        // TODO Auto-generated method stub

    }

}
