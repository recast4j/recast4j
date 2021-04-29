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
package org.recast4j.detour.crowd;

import static org.recast4j.detour.DetourCommon.vCopy;

import java.util.Deque;
import java.util.LinkedList;
import java.util.List;

import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Result;

public class PathQueue {

    private final CrowdConfig config;
    private final Deque<PathQuery> queue = new LinkedList<>();

    protected PathQueue(CrowdConfig config) {
        this.config = config;
    }

    protected void update(NavMesh navMesh) {
        // Update path request until there is nothing to update or up to maxIters pathfinder iterations has been
        // consumed.
        int iterCount = config.maxFindPathIterationsPerUpdate;
        while (iterCount > 0) {
            PathQuery q = queue.poll();
            if (q == null) {
                break;
            }
            // Handle query start.
            if (q.result.status == null) {
                q.navQuery = new NavMeshQuery(navMesh);
                q.result.status = q.navQuery.initSlicedFindPath(q.startRef, q.endRef, q.startPos, q.endPos, q.filter, 0);
            }
            // Handle query in progress.
            if (q.result.status.isInProgress()) {
                Result<Integer> res = q.navQuery.updateSlicedFindPath(iterCount);
                q.result.status = res.status;
                iterCount -= res.result;
            }
            if (q.result.status.isSuccess()) {
                Result<List<Long>> path = q.navQuery.finalizeSlicedFindPath();
                q.result.status = path.status;
                q.result.path = path.result;
            }
            if (!(q.result.status.isFailed() || q.result.status.isSuccess())) {
                queue.addFirst(q);
            }
        }

    }

    protected PathQueryResult request(long startRef, long endRef, float[] startPos, float[] endPos, QueryFilter filter) {
        if (queue.size() >= config.pathQueueSize) {
            return null;
        }
        PathQuery q = new PathQuery();
        vCopy(q.startPos, startPos);
        q.startRef = startRef;
        vCopy(q.endPos, endPos);
        q.endRef = endRef;
        q.result.status = null;
        q.filter = filter;
        queue.add(q);
        return q.result;
    }

}
