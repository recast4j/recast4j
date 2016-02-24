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
