package org.recast4j.detour.extras.jumplink;

import java.util.Deque;
import java.util.LinkedList;

class JumpSegmentBuilder {

    JumpSegment[] build(JumpLinkBuilderConfig acfg, EdgeSampler es) {
        int n = es.end.get(0).gsamples.length;
        int[][] sampleGrid = new int[n][es.end.size()];
        for (int j = 0; j < es.end.size(); j++) {
            for (int i = 0; i < n; i++) {
                sampleGrid[i][j] = -1;
            }
        }
        // Fill connected regions
        int region = 0;
        for (int j = 0; j < es.end.size(); j++) {
            for (int i = 0; i < n; i++) {
                if (sampleGrid[i][j] == -1) {
                    GroundSample p = es.end.get(j).gsamples[i];
                    if (!p.validTrajectory) {
                        sampleGrid[i][j] = -2;
                    } else {
                        Deque<int[]> queue = new LinkedList<>();
                        queue.add(new int[] { i, j });
                        fill(es, sampleGrid, queue, acfg.agentClimb, region);
                        region++;
                    }
                }
            }
        }
        JumpSegment[] jumpSegments = new JumpSegment[region];
        for (int i = 0; i < jumpSegments.length; i++) {
            jumpSegments[i] = new JumpSegment();
        }
        // Find longest segments per region
        for (int j = 0; j < es.end.size(); j++) {
            int l = 0;
            int r = -2;
            for (int i = 0; i < n + 1; i++) {
                if (i == n || sampleGrid[i][j] != r) {
                    if (r >= 0) {
                        if (jumpSegments[r].samples < l) {
                            jumpSegments[r].samples = l;
                            jumpSegments[r].startSample = i - l;
                            jumpSegments[r].groundSegment = j;
                        }
                    }
                    if (i < n) {
                        r = sampleGrid[i][j];
                    }
                    l = 1;
                } else {
                    l++;
                }
            }
        }
        return jumpSegments;
    }

    private void fill(EdgeSampler es, int[][] sampleGrid, Deque<int[]> queue, float agentClimb, int region) {
        while (!queue.isEmpty()) {
            int[] ij = queue.poll();
            int i = ij[0];
            int j = ij[1];
            if (sampleGrid[i][j] == -1) {
                GroundSample p = es.end.get(j).gsamples[i];
                sampleGrid[i][j] = region;
                float h = p.p[1];
                if (i < sampleGrid.length - 1) {
                    addNeighbour(es, queue, agentClimb, h, i + 1, j);
                }
                if (i > 0) {
                    addNeighbour(es, queue, agentClimb, h, i - 1, j);
                }
                if (j < sampleGrid[0].length - 1) {
                    addNeighbour(es, queue, agentClimb, h, i, j + 1);
                }
                if (j > 0) {
                    addNeighbour(es, queue, agentClimb, h, i, j - 1);
                }
            }
        }
    }

    private void addNeighbour(EdgeSampler es, Deque<int[]> queue, float agentClimb, float h, int i, int j) {
        GroundSample q = es.end.get(j).gsamples[i];
        if (q.validTrajectory && Math.abs(q.p[1] - h) < agentClimb) {
            queue.add(new int[] { i, j });
        }
    }

}
