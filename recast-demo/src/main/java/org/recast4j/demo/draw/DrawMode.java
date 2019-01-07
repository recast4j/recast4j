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
package org.recast4j.demo.draw;

public enum DrawMode {
    DRAWMODE_MESH("Input Mesh"),
    DRAWMODE_NAVMESH("Navmesh"),
    DRAWMODE_NAVMESH_INVIS("Navmesh Invis"),
    DRAWMODE_NAVMESH_TRANS("Navmesh Trans"),
    DRAWMODE_NAVMESH_BVTREE("Navmesh BVTree"),
    DRAWMODE_NAVMESH_NODES("Navmesh Nodes"),
    DRAWMODE_NAVMESH_PORTALS("Navmesh Portals"),
    DRAWMODE_VOXELS("Voxels"),
    DRAWMODE_VOXELS_WALKABLE("Walkable Voxels"),
    DRAWMODE_COMPACT("Compact"),
    DRAWMODE_COMPACT_DISTANCE("Compact Distance"),
    DRAWMODE_COMPACT_REGIONS("Compact Regions"),
    DRAWMODE_REGION_CONNECTIONS("Region Connections"),
    DRAWMODE_RAW_CONTOURS("Raw Contours"),
    DRAWMODE_BOTH_CONTOURS("Both Contours"),
    DRAWMODE_CONTOURS("Contours"),
    DRAWMODE_POLYMESH("Poly Mesh"),
    DRAWMODE_POLYMESH_DETAIL("Poly Mesh Detils");
    private final String text;

    private DrawMode(String text) {
        this.text = text;
    }

    @Override
    public String toString() {
        return text;
    }
}
