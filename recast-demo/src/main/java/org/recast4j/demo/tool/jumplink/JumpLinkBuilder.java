package org.recast4j.demo.tool.jumplink;

import static java.util.stream.Collectors.toList;
import static org.recast4j.demo.math.DemoMath.clamp;
import static org.recast4j.demo.math.DemoMath.lerp;
import static org.recast4j.detour.DetourCommon.vCopy;
import static org.recast4j.detour.DetourCommon.vDist2DSqr;
import static org.recast4j.detour.DetourCommon.vLerp;
import static org.recast4j.detour.DetourCommon.vNormalize;
import static org.recast4j.detour.DetourCommon.vSet;
import static org.recast4j.detour.DetourCommon.vSub;
import static org.recast4j.recast.RecastConstants.RC_MESH_NULL_IDX;
import static org.recast4j.recast.RecastConstants.RC_NULL_AREA;

import java.util.ArrayList;
import java.util.List;

import org.recast4j.detour.Tupple2;
import org.recast4j.recast.CompactCell;
import org.recast4j.recast.CompactHeightfield;
import org.recast4j.recast.CompactSpan;
import org.recast4j.recast.Heightfield;
import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.recast4j.recast.Span;

public class JumpLinkBuilder {

    public enum EdgeSamplerType {
        EDGE_JUMP, EDGE_CLIMB_DOWN, EDGE_JUMP_OVER
    }

    private final List<Edge[]> edges;
    private final List<RecastBuilderResult> results;

    public JumpLinkBuilder(List<RecastBuilderResult> results) {
        this.results = results;
        edges = results.stream().map(r -> extractEdges(r.getMesh())).collect(toList());
    }

    private Edge[] extractEdges(PolyMesh mesh) {
        List<Edge> edges = new ArrayList<>();
        float[] orig = mesh.bmin;
        float cs = mesh.cs;
        float ch = mesh.ch;
        for (int i = 0; i < mesh.npolys; i++) { // for (int i = 139; i < 140; i++) { //
            int nvp = mesh.nvp;
            int p = i * 2 * nvp;
            for (int j = 0; j < nvp; ++j) {
                if (mesh.polys[p + j] == RC_MESH_NULL_IDX) {
                    break;
                }
                // Skip connected edges.
                if ((mesh.polys[p + nvp + j] & 0x8000) != 0) {
                    int dir = mesh.polys[p + nvp + j] & 0xf;
                    if (dir == 0xf) {// Border
                        if (mesh.polys[p + nvp + j] != RC_MESH_NULL_IDX) {
                            continue;
                        }
                        int nj = j + 1;
                        if (nj >= nvp || mesh.polys[p + nj] == RC_MESH_NULL_IDX) {
                            nj = 0;
                        }
                        int va = mesh.polys[p + j] * 3;
                        int vb = mesh.polys[p + nj] * 3;
                        Edge e = new Edge();
                        e.sp[0] = orig[0] + mesh.verts[vb] * cs;
                        e.sp[1] = orig[1] + mesh.verts[vb + 1] * ch;
                        e.sp[2] = orig[2] + mesh.verts[vb + 2] * cs;
                        e.sq[0] = orig[0] + mesh.verts[va] * cs;
                        e.sq[1] = orig[1] + mesh.verts[va + 1] * ch;
                        e.sq[2] = orig[2] + mesh.verts[va + 2] * cs;
                        edges.add(e);
                    }
                }
            }
        }
        return edges.toArray(new Edge[edges.size()]);

    }

    public List<JumpLink> build(AnnotationBuilderConfig acfg, EdgeSamplerType type) {
        List<JumpLink> links = new ArrayList<>();
        for (int tile = 0; tile < results.size(); tile++) {
            Edge[] edges = this.edges.get(tile);
            for (int i = 0; i < edges.length; ++i) {
                EdgeSampler edgeSampler = sampleEdge(acfg, results.get(tile), type, edges[i].sp, edges[i].sq);
                if (edgeSampler != null) {
                    links.addAll(addEdgeLinks(acfg, edgeSampler));
                }
            }
        }
        System.out.println("Jump links: " + links.size());
        // filterJumpOverLinks();
        return links;
    }

    EdgeSampler sampleEdge(AnnotationBuilderConfig acfg, RecastBuilderResult result, EdgeSamplerType type, float[] sp,
            float[] sq) {
        EdgeSampler es = new EdgeSampler(type);
        switch (type) {
        case EDGE_JUMP:
            initEdgeJumpRig(acfg, result.getCompactHeightfield(), es, sp, sq);
            break;
        case EDGE_CLIMB_DOWN:
        default:
            initClimbDownRig(acfg, es, sp, sq);
            break;
        case EDGE_JUMP_OVER:
            break;
        }
        /*
        else if (type == EDGE_JUMP_OVER)
        {
            float jumpDist = 4.0f;
            static int NSEGS = 8;
            float segs[NSEGS*6];
            int nsegs = findPotentialJumpOverEdges(sp, sq, jumpDist, 1.0f, segs, NSEGS);
            int ibest = -1;
            float dbest = 0;
            for (int i = 0; i < nsegs; ++i)
            {
                float* seg = &segs[i*6];
                float d = vdistSqr(seg,seg+3);
                if (d > dbest)
                {
                    dbest = d;
                    ibest = i;
                }
            }
            if (ibest == -1)
            {
                delete es;
                return 0;
            }

            initJumpOverRig(es, &segs[ibest*6+0], &segs[ibest*6+3], -jumpDist*0.5f, jumpDist*0.5f, 1.0f, 0.5f);
        }
        */

        // Sample start and end ground segments.
        float dist = (float) Math.sqrt(vDist2DSqr(es.start.p, es.start.q));
        float cs = acfg.cellSize;
        int ngsamples = Math.max(2, (int) Math.ceil(dist / cs));

        sampleGroundSegment(result.getCompactHeightfield(), es.start, ngsamples, es.heightRange);
        sampleGroundSegment(result.getCompactHeightfield(), es.end, ngsamples, es.heightRange);
        es.end.p[1] = es.end.gsamples[0].height;
        es.end.q[1] = es.end.gsamples[es.end.gsamples.length - 1].height;
        sampleAction(acfg, result.getSolidHeightfield(), es);
        return es;
    }

    private void initEdgeJumpRig(AnnotationBuilderConfig acfg, CompactHeightfield chf, EdgeSampler es, float[] sp, float[] sq) {

        vCopy(es.ax, vSub(sq, sp));
        vNormalize(es.ax);
        vSet(es.az, es.ax[2], 0, -es.ax[0]);
        vNormalize(es.az);
        vSet(es.ay, 0, 1, 0);

        // Build action sampling spine.
        es.trajectory.spineYOffset = u -> acfg.groundTolerance;
        es.trajectory.spineYMix = u -> u * u * u;
        es.trajectory.spineXMix = u -> u;

        es.heightRange = acfg.groundRange;

        float cs = acfg.cellSize;

        // Init start end segments.
        float[] offset = new float[3];
        trans2d(offset, es.az, es.ay, new float[] { acfg.startDistance, 0f });
        vadd(es.start.p, sp, offset);
        vadd(es.start.q, sq, offset);

        float de = (float) Math.sqrt(vDist2DSqr(es.start.p, es.start.q));
        int ngsamples = Math.max(2, (int) Math.ceil(de / cs));

        float dx = acfg.endDistance - acfg.agentRadius;
        int nsamples = Math.max(2, (int) Math.ceil(dx / cs));

        for (int i = 0; i < ngsamples; ++i) {
            float u = (float) i / (float) (ngsamples - 1);
            float[] spt = vLerp(sp, sq, u);
            float[] ept = new float[3];
            for (int j = 0; j < nsamples; ++j) {
                float v = (float) i / (float) (nsamples - 1);
                float ox = acfg.agentRadius + dx * v;
                trans2d(offset, es.az, es.ay, new float[] { ox, 0f });
                vadd(ept, spt, offset);
                Tupple2<Boolean, Float> h = getCompactHeightfieldHeight(chf, ept, ept[1] - es.heightRange, ept[1] + es.heightRange);
                if (h.first) {
                    trans2d(offset, es.az, es.ay, new float[] { ox, h.second });
                    vadd(es.end.p, sp, offset);
                    vadd(es.end.q, sq, offset);
                    return;
                }
            }
        }

        trans2d(offset, es.az, es.ay, new float[] { acfg.endDistance, acfg.averageDownDistance });
        vadd(es.end.p, sp, offset);
        vadd(es.end.q, sq, offset);
    }

    private void initClimbDownRig(AnnotationBuilderConfig acfg, EdgeSampler es, float[] sp, float[] sq) {

        vCopy(es.ax, vSub(sq, sp));
        vNormalize(es.ax);
        vSet(es.az, es.ax[2], 0, -es.ax[0]);
        vNormalize(es.az);
        vSet(es.ay, 0, 1, 0);

        // Build action sampling spine.
        es.trajectory.spineYOffset = u -> 0f;
        es.trajectory.spineYMix = u -> Math.max(0f, 2f * u - 1f);
        es.trajectory.spineXMix = u -> Math.min(2f * u, 1f);
        es.heightRange = acfg.groundRange;

        // Init start end segments.
        float[] offset = new float[3];
        trans2d(offset, es.az, es.ay, new float[] { acfg.startDistance, 0f });
        vadd(es.start.p, sp, offset);
        vadd(es.start.q, sq, offset);
        trans2d(offset, es.az, es.ay, new float[] { acfg.endDistance, acfg.averageDownDistance });
        vadd(es.end.p, sp, offset);
        vadd(es.end.q, sq, offset);
    }

    void findPotentialJumpTarget(float[] start, float distance, EdgeSampler sampler) {

    }

    private void sampleGroundSegment(CompactHeightfield chf, GroundSegment seg, int nsamples, float heightRange) {
        seg.ngsamples = nsamples;
        seg.gsamples = new GroundSample[seg.ngsamples];
        seg.npass = 0;

        for (int i = 0; i < seg.ngsamples; ++i) {
            float u = i / (float) (seg.ngsamples - 1);

            GroundSample s = new GroundSample();
            seg.gsamples[i] = s;
            float[] pt = vLerp(seg.p, seg.q, u);
            s.flags = 0;
            float minHeight = pt[1] - heightRange;
            float maxHeight = pt[1] + heightRange;
            Tupple2<Boolean, Float> height = getCompactHeightfieldHeight(chf, pt, minHeight, maxHeight);
            s.height = height.second;

            if (!height.first) {
                continue;
            }
            s.flags |= 1;
            seg.npass++;
        }
    }

    private Tupple2<Boolean, Float> getCompactHeightfieldHeight(CompactHeightfield chf, float[] pt, float minHeight, float maxHeight) {
        float bestHeight = minHeight;
        boolean found = false;
        float range = chf.cs;
        int ix0 = clamp((int) Math.floor((pt[0] - range - chf.bmin[0]) / chf.cs), 0, chf.width - 1);
        int iz0 = clamp((int) Math.floor((pt[2] - range - chf.bmin[2]) / chf.cs), 0, chf.height - 1);
        int ix1 = clamp((int) Math.floor((pt[0] + range - chf.bmin[0]) / chf.cs), 0, chf.width - 1);
        int iz1 = clamp((int) Math.floor((pt[2] + range - chf.bmin[2]) / chf.cs), 0, chf.height - 1);

        for (int z = iz0; z <= iz1; ++z) {
            for (int x = ix0; x <= ix1; ++x) {
                CompactCell c = chf.cells[x + z * chf.width];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    if (s.reg == 0 || chf.areas[i] == RC_NULL_AREA) {
                        continue;
                    }
                    float y = chf.bmin[1] + s.y * chf.ch;
                    if (y > bestHeight && y < maxHeight) {
                        bestHeight = y;
                        found = true;
                    }
                }
            }
        }

        if (found) {
            return new Tupple2<>(true, bestHeight);
        }
        return new Tupple2<>(false, pt[1]);
    }

    private void vadd(float[] dest, float[] v1, float[] v2) {
        dest[0] = v1[0] + v2[0];
        dest[1] = v1[1] + v2[1];
        dest[2] = v1[2] + v2[2];
    }


    /*
     int AnnotationBuilder::findPotentialJumpOverEdges(const float* sp, const float* sq,
                                                  const float depthRange, const float heightRange,
                                                  float* outSegs, const int maxOutSegs)
    {
    // Find potential edges to join to.
    const float widthRange = sqrtf(vdistSqr(sp,sq));
    const float amin[3] = {0,-heightRange*0.5f,0}, amax[3] = {widthRange,heightRange*0.5f,depthRange};

    const float thr = cosf((180.0f - 45.0f)/180.0f*M_PI);

    float ax[3], ay[3], az[3];
    vsub(ax, sq,sp);
    vnormalize(ax);
    vset(az, ax[2], 0, -ax[0]);
    vnormalize(az);
    vset(ay, 0,1,0);

    static const int MAX_SEGS = 64;
    PotentialSeg segs[MAX_SEGS];
    int nsegs = 0;

    for (int i = 0; i < m_nedges; ++i)
    {
        float p[3], lsp[3], lsq[3];
        vsub(p, m_edges[i].sp, sp);
        lsp[0] = ax[0]*p[0] + ay[0]*p[1] + az[0]*p[2];
        lsp[1] = ax[1]*p[0] + ay[1]*p[1] + az[1]*p[2];
        lsp[2] = ax[2]*p[0] + ay[2]*p[1] + az[2]*p[2];

        vsub(p, m_edges[i].sq, sp);
        lsq[0] = ax[0]*p[0] + ay[0]*p[1] + az[0]*p[2];
        lsq[1] = ax[1]*p[0] + ay[1]*p[1] + az[1]*p[2];
        lsq[2] = ax[2]*p[0] + ay[2]*p[1] + az[2]*p[2];

        float tmin, tmax;
        if (isectSegAABB(lsp,lsq, amin,amax, tmin,tmax))
        {
            if (tmin > 1.0f)
                continue;
            if (tmax < 0.0f)
                continue;

            float edir[3];
            vsub(edir, m_edges[i].sq, m_edges[i].sp);
            edir[1] = 0;
            vnormalize(edir);

            if (vdot(ax, edir) > thr)
                continue;

            if (nsegs < MAX_SEGS)
            {
                segs[nsegs].umin = clamp(tmin, 0.0f, 1.0f);
                segs[nsegs].umax = clamp(tmax, 0.0f, 1.0f);
                segs[nsegs].dmin = min(lsp[2],lsq[2]);
                segs[nsegs].dmax = max(lsp[2],lsq[2]);
                segs[nsegs].idx = i;
                segs[nsegs].mark = 0;
                nsegs++;
            }
        }
    }

    const float eps = m_chf->cs;
    unsigned char mark = 1;
    for (int i = 0; i < nsegs; ++i)
    {
        if (segs[i].mark != 0)
            continue;
        segs[i].mark = mark;

        for (int j = i+1; j < nsegs; ++j)
        {
            if (overlapRange(segs[i].dmin-eps, segs[i].dmax+eps,
                             segs[j].dmin-eps, segs[j].dmax+eps))
            {
                segs[j].mark = mark;
            }
        }

        mark++;
    }


    int nout = 0;

    for (int i = 1; i < mark; ++i)
    {
        // Find destination mid point.
        float umin = 10.0f, umax = -10.0;
        float ptmin[3], ptmax[3];

        for (int j = 0; j < nsegs; ++j)
        {
            PotentialSeg* seg = &segs[j];
            if (seg->mark != (unsigned char)i)
                continue;

            float pa[3], pb[3];
            vlerp(pa, m_edges[seg->idx].sp,m_edges[seg->idx].sq, seg->umin);
            vlerp(pb, m_edges[seg->idx].sp,m_edges[seg->idx].sq, seg->umax);

            const float ua = getClosestPtPtSeg(pa, sp, sq);
            const float ub = getClosestPtPtSeg(pb, sp, sq);

            if (ua < umin)
            {
                vcopy(ptmin, pa);
                umin = ua;
            }
            if (ua > umax)
            {
                vcopy(ptmax, pa);
                umax = ua;
            }

            if (ub < umin)
            {
                vcopy(ptmin, pb);
                umin = ub;
            }
            if (ub > umax)
            {
                vcopy(ptmax, pb);
                umax = ub;
            }
        }

        if (umin > umax)
            continue;

        float end[3];
        vlerp(end, ptmin,ptmax,0.5f);

        float start[3];
        vlerp(start, sp,sq,(umin+umax)*0.5f);

        float orig[3];
        vlerp(orig, start,end,0.5f);

        float dir[3], norm[3];
        vsub(dir, end,start);
        dir[1] = 0;
        vnormalize(dir);
        vset(norm, dir[2],0,-dir[0]);

        float ssp[3], ssq[3];

        const float width = widthRange * (umax-umin);
        vmad(ssp, orig, norm, width*0.5f);
        vmad(ssq, orig, norm, -width*0.5f);

        if (nout < maxOutSegs)
        {
            vcopy(&outSegs[nout*6+0], ssp);
            vcopy(&outSegs[nout*6+3], ssq);
            nout++;
        }

    }

    return nout;
    }
     */

    void trans2d(float[] dst, float[] ax, float[] ay, float[] pt) {
        dst[0] = ax[0] * pt[0] + ay[0] * pt[1];
        dst[1] = ax[1] * pt[0] + ay[1] * pt[1];
        dst[2] = ax[2] * pt[0] + ay[2] * pt[1];
    }

    // void trans2dc(float[] dst, float[] c, float[] ax, float[] ay, float[] pt) {
    // dst[0] = c[0] + ax[0] * pt[0] + ay[0] * pt[1];
    // dst[1] = c[1] + ax[1] * pt[0] + ay[1] * pt[1];
    // dst[2] = c[2] + ax[2] * pt[0] + ay[2] * pt[1];
    // }

    private void sampleAction(AnnotationBuilderConfig acfg, Heightfield heightfield, EdgeSampler es) {

        int nsamples = es.start.ngsamples;

        for (int i = 0; i < nsamples; ++i) {
            GroundSample ssmp = es.start.gsamples[i];
            GroundSample esmp = es.end.gsamples[i];

            if ((ssmp.flags & 1) == 0 || (esmp.flags & 1) == 0)
                continue;

            float u = (float) i / (float) (nsamples - 1);
            float[] spt = vLerp(es.start.p, es.start.q, u);
            float[] ept = vLerp(es.end.p, es.end.q, u);

            spt[1] = ssmp.height;
            ept[1] = esmp.height;

            if (!sampleTrajectory(acfg, heightfield, spt, ept, es.trajectory)) {
                continue;
            }
            ssmp.flags |= 4;
        }
    }

    private boolean sampleTrajectory(AnnotationBuilderConfig acfg, Heightfield solid, float[] pa, float[] pb,
            Trajectory2D tra) {
        float cs = acfg.cellSize;
        float dx = acfg.endDistance - acfg.startDistance; // FIXME: Need more vertical samples for climb up/down!
        float dy = Math.abs(pa[1] - pb[1]);
        int nsamples = Math.max(2, (int) Math.ceil(dx / cs));
        for (int i = 0; i < nsamples; ++i) {
            float u = (float) i / (float) (nsamples - 1);
            float x = lerp(pa[0], pb[0], u);
            float y = lerp(pa[1], pb[1], tra.spineYMix.apply(u));
            float z = lerp(pa[2], pb[2], u);
            if (checkHeightfieldCollision(acfg, solid, x, y + acfg.groundTolerance, y + acfg.agentHeight, z)) {
                return false;
            }
        }
        return true;
    }

    private boolean checkHeightfieldCollision(AnnotationBuilderConfig acfg, Heightfield solid, float x, float ymin,
            float ymax, float z) {
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

    private List<JumpLink> addEdgeLinks(AnnotationBuilderConfig acfg, EdgeSampler es) {
        List<JumpLink> links = new ArrayList<>();

        int nsamples = es.start.ngsamples;

        // Filter small holes.
        int RAD = 2;
        int[] kernel = new int[RAD * 2 + 1];

        int[] nflags = new int[nsamples];

        for (int i = 0; i < nsamples; ++i) {
            int a = Math.max(0, i - RAD);
            int b = Math.min(nsamples - 1, i + RAD);
            int nkernel = 0;
            for (int j = a; j <= b; ++j) {
                kernel[nkernel++] = es.start.gsamples[i].flags & 4;
            }
            insertSort(kernel, nkernel);
            nflags[i] = kernel[(nkernel + 1) / 2];
        }

        // Build segments
        int start = -1;
        for (int i = 0; i <= nsamples; ++i) {
            boolean valid = i < nsamples && nflags[i] != 0;
            if (start == -1) {
                if (valid) {
                    start = i;
                }
            } else {
                if (!valid) {
                    float u0 = (float) start / (float) nsamples;
                    float u1 = (float) i / (float) nsamples;
                    float[] sp = vLerp(es.start.p, es.start.q, u0);
                    float[] sq = vLerp(es.start.p, es.start.q, u1);
                    float[] ep = vLerp(es.end.p, es.end.q, u0);
                    float[] eq = vLerp(es.end.p, es.end.q, u1);
                    float sdx = sp[0] - sq[0];
                    float sdz = sp[2] - sq[2];
                    float sd = sdx * sdx + sdz * sdz;
                    float edx = ep[0] - eq[0];
                    float edz = ep[2] - eq[2];
                    float ed = edx * edx + edz * edz;
                    if (Math.max(sd, ed) >= 4 * acfg.agentRadius * acfg.agentRadius) {
                        sp[1] = es.start.gsamples[start].height;
                        sq[1] = es.start.gsamples[i - 1].height;
                        ep[1] = es.end.gsamples[start].height;
                        eq[1] = es.end.gsamples[i - 1].height;

                        JumpLink link = new JumpLink();
                        links.add(link);
                        link.esgeSampler = es;
                        link.nspine = JumpLink.MAX_SPINE;

                        // Build start spine.
                        for (int j = 0; j < link.nspine; ++j) {
                            float spt = ((float) j) / (link.nspine - 1);
                            float u = spt;
                            float[] p = vLerp(sp, ep, es.trajectory.spineXMix.apply(u));
                            float dy = lerp(sp[1], ep[1], es.trajectory.spineYMix.apply(u));
                            p[1] = dy;// + es.trajectory.spineYOffset.apply(u);
                            link.spine0[j * 3] = p[0];
                            link.spine0[j * 3 + 1] = p[1];
                            link.spine0[j * 3 + 2] = p[2];
                        }

                        for (int j = 0; j < link.nspine; ++j) {
                            float spt = ((float) j) / (link.nspine - 1);
                            float u = spt;
                            float[] p = vLerp(sq, eq, es.trajectory.spineXMix.apply(u));
                            float dy = lerp(sq[1], eq[1], es.trajectory.spineYMix.apply(u));
                            p[1] = dy;// + es.trajectory.spineYOffset.apply(u);
                            link.spine1[j * 3] = p[0];
                            link.spine1[j * 3 + 1] = p[1];
                            link.spine1[j * 3 + 2] = p[2];
                        }
                    }

                    start = -1;
                }
            }
        }
        return links;
    }

    private void insertSort(int[] a, int n) {
        int i, j;
        for (i = 1; i < n; i++) {
            int value = a[i];
            for (j = i - 1; j >= 0 && a[j] > value; j--) {
                a[j + 1] = a[j];
            }
            a[j + 1] = value;
        }
    }

    public Edge[] getEdges() {
        return edges.get(0);
    }

}
