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
import java.io.OutputStream;
import java.nio.ByteOrder;

import org.recast4j.detour.io.DetourWriter;
import org.recast4j.detour.io.NavMeshParamWriter;
import org.recast4j.detour.tilecache.CompressedTile;
import org.recast4j.detour.tilecache.TileCache;
import org.recast4j.detour.tilecache.TileCacheBuilder;
import org.recast4j.detour.tilecache.TileCacheLayer;
import org.recast4j.detour.tilecache.TileCacheParams;

public class TileCacheWriter extends DetourWriter {

    private final NavMeshParamWriter paramWriter = new NavMeshParamWriter();
    private final TileCacheBuilder builder = new TileCacheBuilder();

    public void write(OutputStream stream, TileCache cache, ByteOrder order, boolean cCompatibility)
            throws IOException {
        write(stream, TileCacheSetHeader.TILECACHESET_MAGIC, order);
        write(stream, cCompatibility ? TileCacheSetHeader.TILECACHESET_VERSION
                : TileCacheSetHeader.TILECACHESET_VERSION_RECAST4J, order);
        int numTiles = 0;
        for (int i = 0; i < cache.getTileCount(); ++i) {
            CompressedTile tile = cache.getTile(i);
            if (tile == null || tile.data == null)
                continue;
            numTiles++;
        }
        write(stream, numTiles, order);
        paramWriter.write(stream, cache.getNavMesh().getParams(), order);
        writeCacheParams(stream, cache.getParams(), order);
        for (int i = 0; i < cache.getTileCount(); i++) {
            CompressedTile tile = cache.getTile(i);
            if (tile == null || tile.data == null)
                continue;
            write(stream, (int) cache.getTileRef(tile), order);
            byte[] data = tile.data;
            TileCacheLayer layer = cache.decompressTile(tile);
            data = builder.compressTileCacheLayer(layer, order, cCompatibility);
            write(stream, data.length, order);
            stream.write(data);
        }
    }

    private void writeCacheParams(OutputStream stream, TileCacheParams params, ByteOrder order) throws IOException {
        for (int i = 0; i < 3; i++) {
            write(stream, params.orig[i], order);
        }
        write(stream, params.cs, order);
        write(stream, params.ch, order);
        write(stream, params.width, order);
        write(stream, params.height, order);
        write(stream, params.walkableHeight, order);
        write(stream, params.walkableRadius, order);
        write(stream, params.walkableClimb, order);
        write(stream, params.maxSimplificationError, order);
        write(stream, params.maxTiles, order);
        write(stream, params.maxObstacles, order);
    }

}
