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
package org.recast4j.detour.tilecache;

import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public abstract class AbstractTileLayersBuilder {

    protected List<byte[]> build(ByteOrder order, boolean cCompatibility, int threads, int tw, int th) {
        if (threads == 1) {
            return buildSingleThread(order, cCompatibility, tw, th);
        }
        return buildMultiThread(order, cCompatibility, tw, th, threads);
    }

    private List<byte[]> buildSingleThread(ByteOrder order, boolean cCompatibility, int tw, int th) {
        List<byte[]> layers = new ArrayList<>();
        for (int y = 0; y < th; ++y) {
            for (int x = 0; x < tw; ++x) {
                layers.addAll(build(x, y, order, cCompatibility));
            }
        }
        return layers;
    }

    @SuppressWarnings("unchecked")
    private List<byte[]> buildMultiThread(ByteOrder order, boolean cCompatibility, int tw, int th, int threads) {
        ExecutorService ec = Executors.newFixedThreadPool(threads);
        List<?>[][] partialResults = new List[th][tw];
        for (int y = 0; y < th; ++y) {
            for (int x = 0; x < tw; ++x) {
                final int tx = x;
                final int ty = y;
                ec.submit((Runnable) () -> {
                    partialResults[ty][tx] = build(tx, ty, order, cCompatibility);
                });
            }
        }
        ec.shutdown();
        try {
            ec.awaitTermination(1000, TimeUnit.HOURS);
        } catch (InterruptedException e) {
        }
        List<byte[]> layers = new ArrayList<>();
        for (int y = 0; y < th; ++y) {
            for (int x = 0; x < tw; ++x) {
                layers.addAll((List<byte[]>) partialResults[y][x]);
            }
        }
        return layers;
    }

    protected abstract List<byte[]> build(int tx, int ty, ByteOrder order, boolean cCompatibility);
}
