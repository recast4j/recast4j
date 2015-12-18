package org.recast4j.detour.tilecache;

import static org.recast4j.detour.DetourCommon.vCopy;

import java.util.ArrayList;
import java.util.List;

import org.recast4j.recast.HeightfieldLayerSet;
import org.recast4j.recast.HeightfieldLayerSet.HeightfieldLayer;
import org.recast4j.recast.InputGeom;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastConfig;

public class TempObstaclesTest {

	public List<byte[]> rasterizeTileLayers(InputGeom geom, RecastConfig cfg, int tx, int ty) {
		RecastBuilder rcBuilder = new RecastBuilder();
		HeightfieldLayerSet lset = rcBuilder.buildLayers(geom, cfg);
		TileCacheBuilder builder = new TileCacheBuilder();
		List<byte[]> result = new ArrayList<>();
		for (int i = 0; i < lset.layers.length; ++i)
		{
			HeightfieldLayer layer = lset.layers[i];
			
			// Store header
			TileCacheLayerHeader header = new TileCacheLayerHeader();
			header.magic = TileCacheLayerHeader.DT_TILECACHE_MAGIC;
			header.version = TileCacheLayerHeader.DT_TILECACHE_VERSION;
			
			// Tile layer location in the navmesh.
			header.tx = tx;
			header.ty = ty;
			header.tlayer = i;
			vCopy(header.bmin, layer.bmin);
			vCopy(header.bmax, layer.bmax);
			
			// Tile info.
			header.width = layer.width;
			header.height = layer.height;
			header.minx = layer.minx;
			header.maxx = layer.maxx;
			header.miny = layer.miny;
			header.maxy = layer.maxy;
			header.hmin = layer.hmin;
			header.hmax = layer.hmax;
			result.add(builder.buildTileCacheLayer(header));			
	//		dtStatus status = dtBuildTileCacheLayer(&comp, &header, layer.heights, layer.areas, layer.cons,
//													&tile->data, &tile->dataSize);
		}
		return result;
	}
/*
	m_cacheLayerCount = 0;
	m_cacheCompressedSize = 0;
	m_cacheRawSize = 0;
	
	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			TileCacheData tiles[MAX_LAYERS];
			memset(tiles, 0, sizeof(tiles));
			int ntiles = rasterizeTileLayers(m_ctx, m_geom, x, y, cfg, tiles, MAX_LAYERS);

			for (int i = 0; i < ntiles; ++i)
			{
				TileCacheData* tile = &tiles[i];
				status = m_tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
				if (dtStatusFailed(status))
				{
					dtFree(tile->data);
					tile->data = 0;
					continue;
				}
				
				m_cacheLayerCount++;
				m_cacheCompressedSize += tile->dataSize;
				m_cacheRawSize += calcLayerBufferSize(tcparams.width, tcparams.height);
			}
		}
	}
*/

}
