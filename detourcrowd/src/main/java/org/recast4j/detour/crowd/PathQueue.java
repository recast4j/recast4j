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

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;

import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Result;
import org.recast4j.detour.Status;

public class PathQueue {

    static final int DT_PATHQ_INVALID = 0;

    private final PathQuery[] queue;
    private int queueHead;
    private final AtomicLong handleId = new AtomicLong(1);
    private final int maxKeepAlive;
    private NavMeshQuery navQuery;

    protected PathQueue(NavMesh nav, int queueSize, int maxKeepAlive) {
        queue = new PathQuery[queueSize];
        navQuery = new NavMeshQuery(nav);
        this.maxKeepAlive = maxKeepAlive;
        for (int i = 0; i < queue.length; ++i) {
            queue[i] = new PathQuery();
            queue[i].ref = DT_PATHQ_INVALID;
            queue[i].path = new ArrayList<>();
        }
        queueHead = 0;
    }

    protected void update(int maxIters) {
        // Update path request until there is nothing to update
        // or upto maxIters pathfinder iterations has been consumed.
        int iterCount = maxIters;

        for (int i = 0; i < queue.length; ++i) {
            PathQuery q = queue[queueHead % queue.length];

            // Skip inactive requests.
            if (q.ref == DT_PATHQ_INVALID) {
                queueHead++;
                continue;
            }

            // Handle completed request.
            if (q.status != null && (q.status.isSuccess() || q.status.isFailed())) {
                // If the path result has not been read in few frames, free the slot.
                q.keepAlive++;
                if (q.keepAlive > maxKeepAlive) {
                    q.ref = DT_PATHQ_INVALID;
                    q.status = null;
                }

                queueHead++;
                continue;
            }

            // Handle query start.
            if (q.status == null) {
                q.status = navQuery.initSlicedFindPath(q.startRef, q.endRef, q.startPos, q.endPos, q.filter, 0);
            }
            // Handle query in progress.
            if (q.status.isInProgress()) {
                Result<Integer> res = navQuery.updateSlicedFindPath(iterCount);
                q.status = res.status;
                iterCount -= res.result;
            }
            if (q.status.isSuccess()) {
                Result<List<Long>> path = navQuery.finalizeSlicedFindPath();
                q.status = path.status;
                q.path = path.result;
            }

            if (iterCount <= 0)
                break;

            queueHead++;
        }

    }

    protected long request(long startRef, long endRef, float[] startPos, float[] endPos, QueryFilter filter) {
        // Find empty slot
        PathQuery q = null;
        for (PathQuery qi : queue) {
            if (qi.ref == DT_PATHQ_INVALID) {
                q = qi;
                break;
            }
        }
        // Could not find slot.
        if (q == null) {
            return DT_PATHQ_INVALID;
        }

        long ref = handleId.getAndIncrement();
        if (ref == DT_PATHQ_INVALID) {
            ref = handleId.getAndIncrement();
        }

        q.ref = ref;
        vCopy(q.startPos, startPos);
        q.startRef = startRef;
        vCopy(q.endPos, endPos);
        q.endRef = endRef;
        q.status = null;
        q.filter = filter;
        q.keepAlive = 0;
        return ref;

    }

    Status getRequestStatus(long ref) {
        for (PathQuery q : queue) {
            if (q.ref == ref) {
                return q.status;
            }
        }
        return Status.FAILURE;

    }

    Result<List<Long>> getPathResult(long ref) {
        for (PathQuery q : queue) {
            if (q.ref == ref) {
                // Free request for reuse.
                q.ref = DT_PATHQ_INVALID;
                q.status = null;
                return Result.success(q.path);
            }
        }
        return Result.failure();
    }

    public NavMeshQuery getNavQuery() {
        return navQuery;
    }

    public void setNavMesh(NavMesh nav) {
        navQuery = new NavMeshQuery(nav);
    }

}
