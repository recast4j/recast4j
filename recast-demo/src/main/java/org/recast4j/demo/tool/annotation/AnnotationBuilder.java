package org.recast4j.demo.tool.annotation;

import static org.recast4j.detour.DetourCommon.vCopy;
import static org.recast4j.detour.DetourCommon.vDist2DSqr;
import static org.recast4j.detour.DetourCommon.vLerp;
import static org.recast4j.detour.DetourCommon.vMad;
import static org.recast4j.detour.DetourCommon.vNormalize;
import static org.recast4j.detour.DetourCommon.vSet;
import static org.recast4j.detour.DetourCommon.vSub;

import java.util.ArrayList;
import java.util.List;

import org.recast4j.detour.MeshTile;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.Poly;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Result;
import org.recast4j.detour.Tupple2;
import org.recast4j.recast.ContourSet;
import org.recast4j.recast.Heightfield;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.recast4j.recast.Span;

public class AnnotationBuilder {

    public enum EdgeSamplerType {
        EDGE_JUMP_DOWN
    }

    private final Heightfield solid;
    private final Edge[] edges;
    public EdgeSampler edgeSampler;
    private final NavMesh navMesh;
    private final NavMeshQuery navMeshQuery;

    public AnnotationBuilder(List<RecastBuilderResult> result, NavMesh navMesh, NavMeshQuery navMeshQuery) {
        solid = result.get(0).getSolidHeightfield();
        this.navMesh = navMesh;
        this.navMeshQuery = navMeshQuery;
        edges = extractEdges(result.get(0).getContourSet());
    }

    private Edge[] extractEdges(ContourSet cset) {
        List<Edge> edges = new ArrayList<>();
        for (int t = 0; t < navMesh.getTileCount(); t++) {
            MeshTile tile = navMesh.getTile(t);
            if (tile != null) {
                for (int i = 0; i < tile.data.header.polyCount; ++i) {
                    Poly p = tile.data.polys[i];
                    for (int j = 0, nj = p.vertCount; j < nj; ++j) {
                        if (p.neis[j] != 0) {
                            continue;
                        }
                        Edge e = new Edge();
                        e.sp[0] = tile.data.verts[p.verts[(j + 1) % nj] * 3];
                        e.sp[1] = tile.data.verts[p.verts[(j + 1) % nj] * 3 + 1];
                        e.sp[2] = tile.data.verts[p.verts[(j + 1) % nj] * 3 + 2];
                        e.sq[0] = tile.data.verts[p.verts[j] * 3];
                        e.sq[1] = tile.data.verts[p.verts[j] * 3 + 1];
                        e.sq[2] = tile.data.verts[p.verts[j] * 3 + 2];
                        edges.add(e);
                    }
                }
            }
        }
        // System.out.println("me: " + me);
        // float[] orig = cset.bmin;
        // float cs = cset.cs;
        // float ch = cset.ch;
        //
        // for (int i = 0; i < cset.conts.size(); ++i) {
        // Contour c = cset.conts.get(i);
        // if (c.nverts == 0) {
        // continue;
        // }
        //
        // for (int j = 0; j < c.nverts; j++) {
        // int k = (j + 1) % c.nverts;
        // int vj = j * 4;
        // int vk = k * 4;
        //
        // Edge e = new Edge();
        // // edges.add(e);
        //
        // e.sp[0] = orig[0] + c.verts[vk] * cs;
        // e.sp[1] = orig[1] + (c.verts[vk + 1] + 2) * ch;
        // e.sp[2] = orig[2] + c.verts[vk + 2] * cs;
        //
        // e.sq[0] = orig[0] + c.verts[vj] * cs;
        // e.sq[1] = orig[1] + (c.verts[vj + 1] + 2) * ch;
        // e.sq[2] = orig[2] + c.verts[vj + 2] * cs;
        // }
        // }
        return edges.toArray(new Edge[edges.size()]);

    }

    public List<JumpLink> buildAllEdges(AnnotationBuilderConfig acfg, EdgeSamplerType type) {
        List<JumpLink> links = new ArrayList<>();
        for (int i = 0; i < edges.length; ++i) {
            edgeSampler = sampleEdge(acfg, type, edges[i].sp, edges[i].sq);
            if (edgeSampler != null) {
                links.addAll(addEdgeLinks(acfg, edgeSampler));
            }
        }
        System.out.println("Jump links: " + links.size());
        // filterJumpOverLinks();
        return links;
    }

    EdgeSampler sampleEdge(AnnotationBuilderConfig acfg, EdgeSamplerType type, float[] sp, float[] sq) {
        EdgeSampler es = new EdgeSampler();

        if (type == EdgeSamplerType.EDGE_JUMP_DOWN) {
            // initJumpDownRig(es, sp, sq, -0.25f, 2.0f, -3.0f, 0.5f);
            initJumpDownRig(acfg, es, sp, sq);
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
        // Init start end segments.
        float[] offset = new float[3];
        trans2d(offset, es.az, es.ay, new float[] { es.trajectory.spineXOffset.apply(0f),
                es.trajectory.spineYMix.apply(0f) * acfg.jumpDownDistance });
        vadd(es.start.p, es.rigp, offset);
        vadd(es.start.q, es.rigq, offset);
        trans2d(offset, es.az, es.ay, new float[] { es.trajectory.spineXOffset.apply(1f),
                es.trajectory.spineYMix.apply(1f) * acfg.jumpDownDistance });
        vadd(es.end.p, es.rigp, offset);
        vadd(es.end.q, es.rigq, offset);

        // Sample start and end ground segments.
        float dist = (float) Math.sqrt(vDist2DSqr(es.rigp, es.rigq));
        float cs = acfg.cellSize;
        int ngsamples = Math.max(2, (int) Math.ceil(dist / cs));

        sampleGroundSegment(acfg, es.start, ngsamples, es.groundRange);
        sampleGroundSegment(acfg, es.end, ngsamples, es.groundRange);

        sampleAction(acfg, es);
        return es;
    }

    private void sampleGroundSegment(AnnotationBuilderConfig acfg, GroundSegment seg, int nsamples, float groundRange) {
        seg.ngsamples = nsamples;
        seg.gsamples = new GroundSample[seg.ngsamples];
        seg.npass = 0;

        for (int i = 0; i < seg.ngsamples; ++i) {
            float u = i / (float) (seg.ngsamples - 1);

            GroundSample s = new GroundSample();
            seg.gsamples[i] = s;
            float[] pt = vLerp(seg.p, seg.q, u);
            s.flags = 0;
            Tupple2<Boolean, Float> height = getCompactHeightfieldHeight(acfg, pt, groundRange);
            s.height = height.second;

            if (!height.first) {
                continue;
            }
            s.flags |= 1;
            seg.npass++;
        }
    }

    private Tupple2<Boolean, Float> getMeshHeight(float[] pt, float hrange) {
        return null;
    }

    private Tupple2<Boolean, Float> getCompactHeightfieldHeight(AnnotationBuilderConfig acfg, float[] pt,
            float hrange) {
        Result<List<Long>> polys = navMeshQuery.queryPolygons(pt, new float[] { 0.01f, hrange, 0.01f },
                new QueryFilter() {

                    @Override
                    public boolean passFilter(long ref, MeshTile tile, Poly poly) {
                        return true;
                    }

                    @Override
                    public float getCost(float[] pa, float[] pb, long prevRef, MeshTile prevTile, Poly prevPoly,
                            long curRef, MeshTile curTile, Poly curPoly, long nextRef, MeshTile nextTile,
                            Poly nextPoly) {
                        return 0;
                    }
                });
        float bestDist = hrange;
        float bestHeight = Float.MAX_VALUE;
        boolean found = false;

        if (polys.succeeded()) {
            for (long p : polys.result) {
                Result<Float> height = navMeshQuery.getPolyHeight(p, pt);
                if (height.succeeded()) {
                    float dist = Math.abs(height.result - pt[1]);
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestHeight = height.result;
                        found = true;
                    }
                }
            }
        }
        //
        // float range = chf.cs;
        // int ix0 = clamp((int) Math.floor((pt[0] - range - chf.bmin[0]) / chf.cs), 0, chf.width - 1);
        // int iz0 = clamp((int) Math.floor((pt[2] - range - chf.bmin[2]) / chf.cs), 0, chf.height - 1);
        // int ix1 = clamp((int) Math.floor((pt[0] + range - chf.bmin[0]) / chf.cs), 0, chf.width - 1);
        // int iz1 = clamp((int) Math.floor((pt[2] + range - chf.bmin[2]) / chf.cs), 0, chf.height - 1);
        //
        //
        // for (int z = iz0; z <= iz1; ++z) {
        // for (int x = ix0; x <= ix1; ++x) {
        // CompactCell c = chf.cells[x + z * chf.width];
        // for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
        // CompactSpan s = chf.spans[i];
        // if (s.reg == 0 || chf.areas[i] == RC_NULL_AREA)
        // continue;
        // float y = chf.bmin[1] + s.y * chf.ch;
        // float dist = Math.abs(y - pt[1]);
        // if (dist < bestDist) {
        // bestDist = dist;
        // bestHeight = y;
        // found = true;
        // }
        // }
        // }
        // }

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

    private void initJumpDownRig(AnnotationBuilderConfig acfg, EdgeSampler es, float[] sp, float[] sq) {
        vCopy(es.rigp, sp);
        vCopy(es.rigq, sq);

        vCopy(es.ax, vSub(sq, sp));
        vNormalize(es.ax);
        vSet(es.az, es.ax[2], 0, -es.ax[0]);
        vNormalize(es.az);
        vSet(es.ay, 0, 1, 0);

        // Build action sampling spine.
        es.trajectory.spineXOffset = u -> acfg.startDistance + u * (acfg.endDistance - acfg.startDistance);
        es.trajectory.spineXMix = u -> u;
        es.trajectory.spineYMix = u -> u * u * u;

        es.groundRange = acfg.groundRange;
    }

    private int clamp(int v, int min, int max) {
        return Math.max(Math.min(v, max), min);
    }

    private float clamp(float v, float min, float max) {
        return Math.max(Math.min(v, max), min);
    }

    private float lerp(float f, float g, float u) {
        return u * g + (1f - u) * f;
    }

    void trans2d(float[] dst, float[] ax, float[] ay, float[] pt) {
        dst[0] = ax[0] * pt[0] + ay[0] * pt[1];
        dst[1] = ax[1] * pt[0] + ay[1] * pt[1];
        dst[2] = ax[2] * pt[0] + ay[2] * pt[1];
    }

    void trans2dc(float[] dst, float[] c, float[] ax, float[] ay, float[] pt) {
        dst[0] = c[0] + ax[0] * pt[0] + ay[0] * pt[1];
        dst[1] = c[1] + ax[1] * pt[0] + ay[1] * pt[1];
        dst[2] = c[2] + ax[2] * pt[0] + ay[2] * pt[1];
    }

    void sampleAction(AnnotationBuilderConfig acfg, EdgeSampler es) {

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

            if (!sampleTrajectory(acfg, spt, ept, es.trajectory)) {
                continue;
            }
            ssmp.flags |= 4;
        }
    }

    boolean sampleTrajectory(AnnotationBuilderConfig acfg, float[] pa, float[] pb, Trajectory2D tra) {
        float cs = acfg.cellSize;

        float dx = tra.spineXOffset.apply(1f) - tra.spineXOffset.apply(0f);
        int nsamples = Math.max(2, (int) Math.ceil(dx / cs));

        for (int i = 0; i < nsamples; ++i) {
            float u = (float) i / (float) (nsamples - 1);
            float x = tra.spineXMix.apply(u);

            float x0 = clamp(u - acfg.agentRadius / dx, 0f, 1f);
            float x1 = clamp(u + acfg.agentRadius / dx, 0f, 1f);

            float y0 = tra.spineYMix.apply(x0);
            float y1 = tra.spineYMix.apply(x1);

            float ymin = Math.min(y0, y1);
            float ymax = Math.max(y0, y1);

            if (checkHeightfieldCollision(acfg, lerp(pa[0], pb[0], x), lerp(pa[1], pb[1], ymax) + acfg.agentClimb,
                    lerp(pa[1], pb[1], ymin) + acfg.agentHeight, lerp(pa[2], pb[2], x))) {
                return false;
            }
        }

        return true;
    }

    boolean checkHeightfieldCollision(AnnotationBuilderConfig acfg, float x, float ymin, float ymax, float z) {
        // navMeshQuery.queryPolygons(null, null, null)
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

    boolean overlapRange(float amin, float amax, float bmin, float bmax) {
        return (amin > bmax || amax < bmin) ? false : true;
    }

    List<JumpLink> addEdgeLinks(AnnotationBuilderConfig acfg, EdgeSampler es) {
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
                    if (i - start > 1) { // > 5
                        float u0 = (float) start / (float) nsamples;
                        float u1 = (float) i / (float) nsamples;

                        float[] sp = vLerp(es.start.p, es.start.q, u0);
                        float[] sq = vLerp(es.start.p, es.start.q, u1);
                        float[] ep = vLerp(es.end.p, es.end.q, u0);
                        float[] eq = vLerp(es.end.p, es.end.q, u1);
                        sp[1] = es.start.gsamples[start].height;
                        sq[1] = es.start.gsamples[i - 1].height;
                        ep[1] = es.end.gsamples[start].height;
                        eq[1] = es.end.gsamples[i - 1].height;

                        JumpLink link = new JumpLink();
                        links.add(link);

                        link.flags = 1;
                        link.nspine = JumpLink.MAX_SPINE;

                        // Build start spine.
                        for (int j = 0; j < link.nspine; ++j) {
                            float spt = ((float) j) / (link.nspine - 1);
                            float u = es.trajectory.spineXMix.apply(spt);
                            float dy = lerp(sp[1], ep[1], es.trajectory.spineYMix.apply(spt));
                            float[] p = vLerp(sp, ep, u);
                            p[1] = acfg.agentClimb;
                            p = vMad(p, es.ay, dy);
                            link.spine0[j * 3] = p[0];
                            link.spine0[j * 3 + 1] = p[1];
                            link.spine0[j * 3 + 2] = p[2];
                        }

                        for (int j = 0; j < link.nspine; ++j) {
                            float spt = ((float) j) / (link.nspine - 1);
                            float u = es.trajectory.spineXMix.apply(spt);
                            float dy = lerp(sq[1], eq[1], es.trajectory.spineYMix.apply(spt));
                            float[] p = vLerp(sq, eq, u);
                            p[1] = acfg.agentClimb;
                            p = vMad(p, es.ay, dy);
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

    static void insertSort(int[] a, int n) {
        int i, j;
        for (i = 1; i < n; i++) {
            int value = a[i];
            for (j = i - 1; j >= 0 && a[j] > value; j--)
                a[j + 1] = a[j];
            a[j + 1] = value;
        }
    }

    public Edge[] getEdges() {
        return edges;
    }

}
