package org.recast4j.detour.extras.unity.astar;

import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.Arrays;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

import com.google.gson.GsonBuilder;

class MetaReader {

	public Meta read(ZipFile file, String filename) throws IOException {
		ZipEntry entry = file.getEntry(filename);
		try (Reader r = new InputStreamReader(file.getInputStream(entry))) {
			Meta meta = new GsonBuilder().create().fromJson(r, Meta.class);
			if (!meta.isSupportedType()) {
				throw new IllegalArgumentException("Unsupported graph type " + Arrays.toString(meta.typeNames));
			}
			if (!meta.isSupportedVersion()) {
				throw new IllegalArgumentException("Unsupported version " + meta.version);
			}
			return meta;
		}
	}

}
