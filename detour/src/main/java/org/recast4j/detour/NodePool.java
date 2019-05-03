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
package org.recast4j.detour;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class NodePool {

    private final Map<Long, List<Node>> m_map = new HashMap<>();
    private final ArrayList<Node> m_nodes = new ArrayList<>();

    public NodePool() {

    }

    public void clear() {
        m_nodes.clear();
        m_map.clear();
    }

    List<Node> findNodes(long id) {
        List<Node> nodes = m_map.get(id);
        if (nodes == null) {
            nodes = new ArrayList<>();
        }
        return nodes;
    }

    Node findNode(long id) {
        List<Node> nodes = m_map.get(id);
        if (nodes != null && !nodes.isEmpty()) {
            return nodes.get(0);
        }
        return null;
    }

    Node findNode(long id, int state) {
        List<Node> nodes = m_map.get(id);
        if (nodes != null) {
            for (Node node : nodes) {
                if (node.state == state) {
                    return node;
                }
            }
        }
        return null;
    }

    Node getNode(long id, int state) {
        List<Node> nodes = m_map.get(id);
        if (nodes != null) {
            for (Node node : nodes) {
                if (node.state == state) {
                    return node;
                }
            }
        }
        return create(id, state);
    }

    protected Node create(long id, int state) {
        Node node = new Node(m_nodes.size() + 1);
        node.id = id;
        node.state = state;
        m_nodes.add(node);
        List<Node> nodes = m_map.get(id);
        if (nodes == null) {
            nodes = new ArrayList<>();
            m_map.put(id, nodes);
        }
        nodes.add(node);
        return node;
    }

    public int getNodeIdx(Node node) {
        return node != null ? node.index : 0;
    }

    public Node getNodeAtIdx(int idx) {
        return idx != 0 ? m_nodes.get(idx - 1) : null;
    }

    public int getNodeCount() {
        return m_nodes.size();
    }

    public Node getNode(long ref) {
        return getNode(ref, 0);
    }

    public Map<Long, List<Node>> getNodeMap() {
        return m_map;
    }

}
