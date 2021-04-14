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

package org.recast4j.detour.extras.unity.astar;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.ZipFile;

public class UnityAStarPathfindingReader {

    private static final String META_FILE_NAME = "meta.json";
    private static final String NODE_INDEX_FILE_NAME = "graph_references.binary";
    private static final String NODE_LINK_2_FILE_NAME = "node_link2.binary";
    private static final String GRAPH_META_FILE_NAME_PATTERN = "graph%d.json";
    private static final String GRAPH_DATA_FILE_NAME_PATTERN = "graph%d_extra.binary";
    private static final String GRAPH_CONNECTION_FILE_NAME_PATTERN = "graph%d_references.binary";
    private static final int MAX_VERTS_PER_POLY = 3;
    private final MetaReader metaReader = new MetaReader();
    private final NodeIndexReader nodeIndexReader = new NodeIndexReader();
    private final GraphMetaReader graphMetaReader = new GraphMetaReader();
    private final GraphMeshDataReader graphDataReader = new GraphMeshDataReader();
    private final GraphConnectionReader graphConnectionReader = new GraphConnectionReader();
    private final NodeLink2Reader nodeLink2Reader = new NodeLink2Reader();

    public GraphData read(File zipFile) throws Exception {
        ZipFile file = new ZipFile(zipFile);
        // Read meta file and check version and graph type
        Meta meta = metaReader.read(file, META_FILE_NAME);
        // Read index to node mapping
        int[] indexToNode = nodeIndexReader.read(file, NODE_INDEX_FILE_NAME);
        // Read NodeLink2 data (off-mesh links)
        NodeLink2[] nodeLinks2 = nodeLink2Reader.read(file, NODE_LINK_2_FILE_NAME, indexToNode);
        // Read graph by graph
        List<GraphMeta> metaList = new ArrayList<>();
        List<GraphMeshData> meshDataList = new ArrayList<>();
        List<List<int[]>> connectionsList = new ArrayList<>();
        for (int graphIndex = 0; graphIndex < meta.graphs; graphIndex++) {
            GraphMeta graphMeta = graphMetaReader.read(file, String.format(GRAPH_META_FILE_NAME_PATTERN, graphIndex));
            // First graph mesh data - vertices and polygons
            GraphMeshData graphData = graphDataReader.read(file,
                    String.format(GRAPH_DATA_FILE_NAME_PATTERN, graphIndex), graphMeta, MAX_VERTS_PER_POLY);
            // Then graph connection data - links between nodes located in both the same tile and other tiles
            List<int[]> connections = graphConnectionReader.read(file,
                    String.format(GRAPH_CONNECTION_FILE_NAME_PATTERN, graphIndex), meta, indexToNode);
            metaList.add(graphMeta);
            meshDataList.add(graphData);
            connectionsList.add(connections);
        }
        file.close();
        return new GraphData(meta, indexToNode, nodeLinks2, metaList, meshDataList, connectionsList);
    }
}
