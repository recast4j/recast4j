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
package org.recast4j.detour.tilecache.io;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.recast4j.detour.NavMesh;
import org.recast4j.detour.io.IOUtils;
import org.recast4j.detour.io.NavMeshParamReader;
import org.recast4j.detour.tilecache.TileCache;
import org.recast4j.detour.tilecache.TileCacheCompressor;
import org.recast4j.detour.tilecache.TileCacheMeshProcess;
import org.recast4j.detour.tilecache.TileCacheParams;
import org.recast4j.detour.tilecache.TileCacheStorageParams;
import org.recast4j.detour.tilecache.io.compress.TileCacheCompressorFactory;

public class TileCacheReader {

    private final NavMeshParamReader paramReader = new NavMeshParamReader();

    public TileCache read(InputStream is, int maxVertPerPoly, TileCacheMeshProcess meshProcessor) throws IOException {
        ByteBuffer bb = IOUtils.toByteBuffer(is);
        return read(bb, maxVertPerPoly, meshProcessor);
    }

    public TileCache read(ByteBuffer bb, int maxVertPerPoly, TileCacheMeshProcess meshProcessor) throws IOException {
        TileCacheSetHeader header = new TileCacheSetHeader();
        header.magic = bb.getInt();
        if (header.magic != TileCacheSetHeader.TILECACHESET_MAGIC) {
            header.magic = IOUtils.swapEndianness(header.magic);
            if (header.magic != TileCacheSetHeader.TILECACHESET_MAGIC) {
                throw new IOException("Invalid magic");
            }
            bb.order(bb.order() == ByteOrder.BIG_ENDIAN ? ByteOrder.LITTLE_ENDIAN : ByteOrder.BIG_ENDIAN);
        }
        header.version = bb.getInt();
        if (header.version != TileCacheSetHeader.TILECACHESET_VERSION) {
            if (header.version != TileCacheSetHeader.TILECACHESET_VERSION_RECAST4J) {
                throw new IOException("Invalid version");
            }
        }
        boolean cCompatibility = header.version == TileCacheSetHeader.TILECACHESET_VERSION;
        header.numTiles = bb.getInt();
        header.meshParams = paramReader.read(bb);
        header.cacheParams = readCacheParams(bb, cCompatibility);
        NavMesh mesh = new NavMesh(header.meshParams, maxVertPerPoly);
        TileCacheCompressor compressor = TileCacheCompressorFactory.get(cCompatibility);
        TileCache tc = new TileCache(header.cacheParams, new TileCacheStorageParams(bb.order(), cCompatibility), mesh,
                compressor, meshProcessor);
        // Read tiles.
        for (int i = 0; i < header.numTiles; ++i) {
            long tileRef = bb.getInt();
            int dataSize = bb.getInt();
            if (tileRef == 0 || dataSize == 0) {
                break;
            }
            byte[] data = new byte[dataSize];
            bb.get(data);
            long tile = tc.addTile(data, 0);
            if (tile != 0) {
                tc.buildNavMeshTile(tile);
            }
        }
        return tc;
    }

    private TileCacheParams readCacheParams(ByteBuffer bb, boolean cCompatibility) {
        TileCacheParams params = new TileCacheParams();
        for (int i = 0; i < 3; i++) {
            params.orig[i] = bb.getFloat();
        }
        params.cs = bb.getFloat();
        params.ch = bb.getFloat();
        params.width = bb.getInt();
        params.height = bb.getInt();
        params.walkableHeight = bb.getFloat();
        params.walkableRadius = bb.getFloat();
        params.walkableClimb = bb.getFloat();
        params.maxSimplificationError = bb.getFloat();
        params.maxTiles = bb.getInt();
        params.maxObstacles = bb.getInt();
        return params;
    }
}
