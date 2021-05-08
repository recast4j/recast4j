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

package org.recast4j.detour.crowd;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class CrowdTelemetry {

    private static final int TIMING_SAMPLES = 10;
    private float maxTimeToEnqueueRequest;
    private float maxTimeToFindPath;
    private final Map<String, Long> executionTimings = new HashMap<>();
    private final Map<String, List<Long>> executionTimingSamples = new HashMap<>();

    public float maxTimeToEnqueueRequest() {
        return maxTimeToEnqueueRequest;
    }

    public float maxTimeToFindPath() {
        return maxTimeToFindPath;
    }

    public Map<String, Long> executionTimings() {
        return executionTimings;
    }

    void start() {
        maxTimeToEnqueueRequest = 0;
        maxTimeToFindPath = 0;
        executionTimings.clear();
    }

    void recordMaxTimeToEnqueueRequest(float time) {
        maxTimeToEnqueueRequest = Math.max(maxTimeToEnqueueRequest, time);
    }

    void recordMaxTimeToFindPath(float time) {
        maxTimeToFindPath = Math.max(maxTimeToFindPath, time);
    }

    void start(String name) {
        executionTimings.put(name, System.nanoTime());
    }

    void stop(String name) {
        long duration = System.nanoTime() - executionTimings.get(name);
        List<Long> s = executionTimingSamples.computeIfAbsent(name, __ -> new ArrayList<>());
        if (s.size() == TIMING_SAMPLES) {
            s.remove(0);
        }
        s.add(duration);
        executionTimings.put(name, (long) s.stream().mapToLong(l -> l).average().orElse(0));
    }
}
