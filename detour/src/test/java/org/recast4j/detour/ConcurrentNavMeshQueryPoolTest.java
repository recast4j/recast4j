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

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.recast4j.detour.ConcurrentNavMeshQueryPool.PooledNavMeshQuery;

public class ConcurrentNavMeshQueryPoolTest {

    private ConcurrentNavMeshQueryPool pool;
    private ExecutorService executor;

    @Before
    public void setUp() {
        pool = new ConcurrentNavMeshQueryPool(null, 4);
        executor = Executors.newFixedThreadPool(16);
    }

    @After
    public void tearDown() {
        executor.shutdownNow();
    }

    @Test
    public void shouldReuseQueries() throws InterruptedException, ExecutionException {
        Set<NavMeshQuery> queries = ConcurrentHashMap.newKeySet();
        List<CompletableFuture<?>> futures = new ArrayList<>();
        for (int i = 0; i < 32; i++) {
            futures.add(CompletableFuture.runAsync(() -> {
                try (PooledNavMeshQuery q = pool.acquire()) {
                    queries.add(q.query());
                }
            }, executor));
        }
        CompletableFuture.allOf(futures.stream().toArray(CompletableFuture[]::new)).get();
        assertThat(queries).hasSize(4);
    }
}
