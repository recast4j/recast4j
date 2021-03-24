package org.recast4j.recast;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicLong;

public class Telemetry {

    private ThreadLocal<Map<String, AtomicLong>> timerStart = ThreadLocal.withInitial(HashMap::new);
    private Map<String, AtomicLong> timerAccum = new ConcurrentHashMap<>();

    public void startTimer(String name) {
        timerStart.get().put(name, new AtomicLong(System.nanoTime()));
    }

    public void stopTimer(String name) {
        timerAccum.computeIfAbsent(name, __ -> new AtomicLong()).addAndGet(System.nanoTime() - timerStart.get().get(name).get());
    }

    public void warn(String string) {
        System.err.println(string);
    }

    public void print() {
        timerAccum.forEach((n, v) -> System.out.println(n + ": " + v.get() / 1000000));
    }


}
