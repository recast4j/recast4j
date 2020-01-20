package org.recast4j.detour.extras.jumplink;

import static org.recast4j.detour.DetourCommon.vDist2DSqr;
import static org.recast4j.detour.DetourCommon.vLerp;

import java.util.function.BiFunction;

import org.recast4j.detour.Tupple2;

abstract class AbstractGroundSampler implements GroundSampler {

    protected void sampleGround(JumpLinkBuilderConfig acfg, EdgeSampler es,
            BiFunction<float[], Float, Tupple2<Boolean, Float>> heightFunc) {
        float cs = acfg.cellSize;
        float dist = (float) Math.sqrt(vDist2DSqr(es.start.p, es.start.q));
        int ngsamples = Math.max(2, (int) Math.ceil(dist / cs));
        sampleGroundSegment(heightFunc, es.start, ngsamples);
        for (GroundSegment end : es.end) {
            sampleGroundSegment(heightFunc, end, ngsamples);
        }
    }

    protected void sampleGroundSegment(BiFunction<float[], Float, Tupple2<Boolean, Float>> heightFunc, GroundSegment seg,
            int nsamples) {
        seg.gsamples = new GroundSample[nsamples];

        for (int i = 0; i < nsamples; ++i) {
            float u = i / (float) (nsamples - 1);

            GroundSample s = new GroundSample();
            seg.gsamples[i] = s;
            float[] pt = vLerp(seg.p, seg.q, u);
            Tupple2<Boolean, Float> height = heightFunc.apply(pt, seg.height);
            s.p[0] = pt[0];
            s.p[1] = height.second;
            s.p[2] = pt[2];

            if (!height.first) {
                continue;
            }
            s.validHeight = true;
        }
    }

}
