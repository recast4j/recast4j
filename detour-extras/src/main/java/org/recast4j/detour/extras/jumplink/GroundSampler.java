package org.recast4j.detour.extras.jumplink;

import org.recast4j.recast.RecastBuilder.RecastBuilderResult;

public interface GroundSampler {

    void sample(JumpLinkBuilderConfig acfg, RecastBuilderResult result, EdgeSampler es);

}
