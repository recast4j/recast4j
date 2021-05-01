/*
recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org

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

package org.recast4j.detour;

import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Simple object pool which can be used to provide the given number of shared NavMeshQuery instances for concurrent
 * execution
 */
public class ConcurrentNavMeshQueryPool {

    private final NavMesh navMesh;
    private final AtomicInteger remaining;
    private final LinkedBlockingQueue<PooledNavMeshQuery> queue = new LinkedBlockingQueue<>();

    public ConcurrentNavMeshQueryPool(NavMesh navMesh, int poolSize) {
        this.navMesh = navMesh;
        remaining = new AtomicInteger(poolSize);
    }

    public PooledNavMeshQuery acquire() {
        PooledNavMeshQuery query = queue.poll();
        if (query == null) {
            if (remaining.getAndDecrement() > 0) {
                query = new PooledNavMeshQuery(new NavMeshQuery(navMesh));
            } else {
                try {
                    query = queue.take();
                } catch (InterruptedException e) {
                }
            }
        }
        return query;
    }

    public class PooledNavMeshQuery implements AutoCloseable {

        private final NavMeshQuery query;

        public PooledNavMeshQuery(NavMeshQuery query) {
            this.query = query;
        }

        @Override
        public void close() {
            queue.offer(this);
        }

        public NavMeshQuery query() {
            return query;
        }
    }
}
