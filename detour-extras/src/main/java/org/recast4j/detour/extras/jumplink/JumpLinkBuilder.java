package org.recast4j.detour.extras.jumplink;

import static java.util.stream.Collectors.toList;
import static org.recast4j.detour.DetourCommon.vDist2DSqr;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.recast4j.recast.RecastBuilder.RecastBuilderResult;

public class JumpLinkBuilder {

    private final EdgeExtractor edgeExtractor = new EdgeExtractor();
    private final EdgeSamplerFactory edgeSamplerFactory = new EdgeSamplerFactory();
    private final GroundSampler groundSampler = new NavMeshGroundSampler();
    private final TrajectorySampler trajectorySampler = new TrajectorySampler();
    private final JumpSegmentBuilder jumpSegmentBuilder = new JumpSegmentBuilder();

    private final List<Edge[]> edges;
    private final List<RecastBuilderResult> results;

    public JumpLinkBuilder(List<RecastBuilderResult> results) {
        this.results = results;
        edges = results.stream().map(r -> edgeExtractor.extractEdges(r.getMesh())).collect(toList());
    }

    public List<JumpLink> build(JumpLinkBuilderConfig acfg, JumpLinkType type) {
        List<JumpLink> links = new ArrayList<>();
        for (int tile = 0; tile < results.size(); tile++) {
            Edge[] edges = this.edges.get(tile);
            for (Edge edge : edges) {
                links.addAll(processEdge(acfg, results.get(tile), type, edge));
            }
        }
        return links;
    }

    private List<JumpLink> processEdge(JumpLinkBuilderConfig acfg, RecastBuilderResult result, JumpLinkType type, Edge edge) {
        EdgeSampler es = edgeSamplerFactory.get(acfg, type, edge);
        groundSampler.sample(acfg, result, es);
        trajectorySampler.sample(acfg, result.getSolidHeightfield(), es);
        JumpSegment[] jumpSegments = jumpSegmentBuilder.build(acfg, es);
        return buildJumpLinks(acfg, es, jumpSegments);
    }


    private List<JumpLink> buildJumpLinks(JumpLinkBuilderConfig acfg, EdgeSampler es, JumpSegment[] jumpSegments) {
        List<JumpLink> links = new ArrayList<>();
        for (JumpSegment js : jumpSegments) {
            float[] sp = es.start.gsamples[js.startSample].p;
            float[] sq = es.start.gsamples[js.startSample + js.samples - 1].p;
            GroundSegment end = es.end.get(js.groundSegment);
            float[] ep = end.gsamples[js.startSample].p;
            float[] eq = end.gsamples[js.startSample + js.samples - 1].p;
            float d = Math.min(vDist2DSqr(sp, sq), vDist2DSqr(ep, eq));
            if (d >= 4 * acfg.agentRadius * acfg.agentRadius) {
                JumpLink link = new JumpLink();
                links.add(link);
                link.startSamples = Arrays.copyOfRange(es.start.gsamples, js.startSample, js.startSample + js.samples);
                link.endSamples = Arrays.copyOfRange(end.gsamples, js.startSample, js.startSample + js.samples);
                link.start = es.start;
                link.end = end;
                link.trajectory = es.trajectory;
                for (int j = 0; j < link.nspine; ++j) {
                    float u = ((float) j) / (link.nspine - 1);
                    float[] p = es.trajectory.apply(sp, ep, u);
                    link.spine0[j * 3] = p[0];
                    link.spine0[j * 3 + 1] = p[1];
                    link.spine0[j * 3 + 2] = p[2];

                    p = es.trajectory.apply(sq, eq, u);
                    link.spine1[j * 3] = p[0];
                    link.spine1[j * 3 + 1] = p[1];
                    link.spine1[j * 3 + 2] = p[2];
                }
            }
        }
        return links;
    }

    public List<Edge[]> getEdges() {
        return edges;
    }

}
