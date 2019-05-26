/*
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
package org.recast4j.detour.extras.unity.astar;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.ZipFile;

class GraphConnectionReader extends BinaryReader {

    @SuppressWarnings("unused")
    List<int[]> read(ZipFile file, String filename, Meta meta, int[] indexToNode) throws IOException {
        List<int[]> connections = new ArrayList<>();
        ByteBuffer buffer = toByteBuffer(file, filename);
        while (buffer.remaining() > 0) {
            int count = buffer.getInt();
            int[] nodeConnections = new int[count];
            connections.add(nodeConnections);
            for (int i = 0; i < count; i++) {
                int nodeIndex = buffer.getInt();
                nodeConnections[i] = indexToNode[nodeIndex];
                // XXX: Is there anything we can do with the cost?
                int cost = buffer.getInt();
                if (meta.isSupportedVersion(Meta.UPDATED_STRUCT_VERSION)) {
                    byte shapeEdge = buffer.get();
                }
            }
        }
        return connections;
    }

}
