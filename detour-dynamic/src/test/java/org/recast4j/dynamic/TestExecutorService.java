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

package org.recast4j.dynamic;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicLong;

class TestExecutorService {

    private static final ExecutorService EXECUTOR = Executors
            .newFixedThreadPool(Math.max(1, Runtime.getRuntime().availableProcessors() / 2), new DaemonThreadFactory());

    public static ExecutorService get() {
        return EXECUTOR;
    }

    private static class DaemonThreadFactory implements ThreadFactory {
        private final AtomicLong count = new AtomicLong();

        @Override
        public Thread newThread(Runnable r) {
            Thread t = new Thread(r);
            t.setDaemon(true);
            t.setName("DynamicNavMeshTest - " + count.getAndIncrement());
            return t;
        }
    }
}