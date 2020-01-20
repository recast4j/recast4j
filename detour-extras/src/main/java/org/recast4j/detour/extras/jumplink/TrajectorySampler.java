package org.recast4j.detour.extras.jumplink;

import static org.recast4j.detour.DetourCommon.vDist2D;

import org.recast4j.recast.Heightfield;
import org.recast4j.recast.Span;

class TrajectorySampler {

    void sample(JumpLinkBuilderConfig acfg, Heightfield heightfield, EdgeSampler es) {
        int nsamples = es.start.gsamples.length;
        for (int i = 0; i < nsamples; ++i) {
            GroundSample ssmp = es.start.gsamples[i];
            for (GroundSegment end : es.end) {
                GroundSample esmp = end.gsamples[i];
                if (!ssmp.validHeight || !esmp.validHeight) {
                    continue;
                }

                if (!sampleTrajectory(acfg, heightfield, ssmp.p, esmp.p, es.trajectory)) {
                    continue;
                }
                ssmp.validTrajectory = true;
                esmp.validTrajectory = true;
            }
        }
    }

    private boolean sampleTrajectory(JumpLinkBuilderConfig acfg, Heightfield solid, float[] pa, float[] pb, Trajectory tra) {
        float cs = Math.min(acfg.cellSize, acfg.cellHeight);
        float d = vDist2D(pa, pb) + Math.abs(pa[1] - pb[1]);
        int nsamples = Math.max(2, (int) Math.ceil(d / cs));
        for (int i = 0; i < nsamples; ++i) {
            float u = (float) i / (float) (nsamples - 1);
            float[] p = tra.apply(pa, pb, u);
            if (checkHeightfieldCollision(solid, p[0], p[1] + acfg.groundTolerance, p[1] + acfg.agentHeight, p[2])) {
                return false;
            }
        }
        return true;
    }

    private boolean checkHeightfieldCollision(Heightfield solid, float x, float ymin, float ymax, float z) {
        int w = solid.width;
        int h = solid.height;
        float cs = solid.cs;
        float ch = solid.ch;
        float[] orig = solid.bmin;
        int ix = (int) Math.floor((x - orig[0]) / cs);
        int iz = (int) Math.floor((z - orig[2]) / cs);

        if (ix < 0 || iz < 0 || ix > w || iz > h) {
            return false;
        }

        Span s = solid.spans[ix + iz * w];
        if (s == null) {
            return false;
        }

        while (s != null) {
            float symin = orig[1] + s.smin * ch;
            float symax = orig[1] + s.smax * ch;
            if (overlapRange(ymin, ymax, symin, symax)) {
                return true;
            }
            s = s.next;
        }

        return false;
    }

    private boolean overlapRange(float amin, float amax, float bmin, float bmax) {
        return (amin > bmax || amax < bmin) ? false : true;
    }

}
