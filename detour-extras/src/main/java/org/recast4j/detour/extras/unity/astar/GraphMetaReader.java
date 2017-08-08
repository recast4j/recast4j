package org.recast4j.detour.extras.unity.astar;

import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

import com.google.gson.GsonBuilder;

class GraphMetaReader {

	GraphMeta read(ZipFile file, String filename) throws IOException {
		ZipEntry entry = file.getEntry(filename);
		try (Reader r = new InputStreamReader(file.getInputStream(entry))) {
			return new GsonBuilder().create().fromJson(r, GraphMeta.class);
		}
	}
}
