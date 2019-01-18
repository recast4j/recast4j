/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j Copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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

import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Result;
import org.recast4j.detour.Status;

public class PathQueue {

    private static final int MAX_QUEUE = 8;
    static final int DT_PATHQ_INVALID = 0;
    private static final int MAX_KEEP_ALIVE = 2; // in update ticks.

    private final PathQuery[] m_queue = new PathQuery[MAX_QUEUE];
    private long m_nextHandle = 1;
    private int m_queueHead;
    private final NavMeshQuery m_navquery;

    protected PathQueue(int maxSearchNodeCount, NavMesh nav) {
        m_navquery = new NavMeshQuery(nav);
        for (int i = 0; i < MAX_QUEUE; ++i) {
            m_queue[i] = new PathQuery();
            m_queue[i].ref = DT_PATHQ_INVALID;
            m_queue[i].path = new ArrayList<>(256);
        }
        m_queueHead = 0;
    }

    protected void update(int maxIters) {
        // Update path request until there is nothing to update
        // or upto maxIters pathfinder iterations has been consumed.
        int iterCount = maxIters;

        for (int i = 0; i < MAX_QUEUE; ++i) {
            PathQuery q = m_queue[m_queueHead % MAX_QUEUE];

            // Skip inactive requests.
            if (q.ref == DT_PATHQ_INVALID) {
                m_queueHead++;
                continue;
            }

            // Handle completed request.
            if (q.status != null && (q.status.isSuccess() || q.status.isFailed())) {
                // If the path result has not been read in few frames, free the slot.
                q.keepAlive++;
                if (q.keepAlive > MAX_KEEP_ALIVE) {
                    q.ref = DT_PATHQ_INVALID;
                    q.status = null;
                }

                m_queueHead++;
                continue;
            }

            // Handle query start.
            if (q.status == null) {
                q.status = m_navquery.initSlicedFindPath(q.startRef, q.endRef, q.startPos, q.endPos, q.filter, 0);
            }
            // Handle query in progress.
            if (q.status.isInProgress()) {
                int iters = 0;
                Result<Integer> res = m_navquery.updateSlicedFindPath(iterCount);
                iters = res.result;
                q.status = res.status;
                iterCount -= iters;
            }
            if (q.status.isSuccess()) {
                Result<List<Long>> path = m_navquery.finalizeSlicedFindPath();
                q.status = path.status;
                q.path = path.result;
            }

            if (iterCount <= 0)
                break;

            m_queueHead++;
        }

    }

    protected long request(long startRef, long endRef, float[] startPos, float[] endPos, QueryFilter filter) {
        // Find empty slot
        int slot = -1;
        for (int i = 0; i < MAX_QUEUE; ++i) {
            if (m_queue[i].ref == DT_PATHQ_INVALID) {
                slot = i;
                break;
            }
        }
        // Could not find slot.
        if (slot == -1)
            return DT_PATHQ_INVALID;

        long ref = m_nextHandle++;
        if (m_nextHandle == DT_PATHQ_INVALID)
            m_nextHandle++;

        PathQuery q = m_queue[slot];
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
        for (int i = 0; i < MAX_QUEUE; ++i) {
            if (m_queue[i].ref == ref)
                return m_queue[i].status;
        }
        return Status.FAILURE;

    }

    Result<List<Long>> getPathResult(long ref) {
        for (int i = 0; i < MAX_QUEUE; ++i) {
            if (m_queue[i].ref == ref) {
                PathQuery q = m_queue[i];
                // Free request for reuse.
                q.ref = DT_PATHQ_INVALID;
                q.status = null;
                return Result.success(q.path);
            }
        }
        return Result.failure();
    }

    public NavMeshQuery getNavQuery() {
        return m_navquery;
    }
}
