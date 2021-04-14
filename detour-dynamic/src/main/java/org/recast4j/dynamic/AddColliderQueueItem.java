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

package org.recast4j.dynamic;

import java.util.Collection;

import org.recast4j.dynamic.collider.Collider;

public class AddColliderQueueItem implements UpdateQueueItem {

    private final long colliderId;
    private final Collider collider;
    private final Collection<DynamicTile> affectedTiles;

    public AddColliderQueueItem(long colliderId, Collider collider, Collection<DynamicTile> affectedTiles) {
        this.colliderId = colliderId;
        this.collider = collider;
        this.affectedTiles = affectedTiles;
    }

    @Override
    public Collection<DynamicTile> affectedTiles() {
        return affectedTiles;
    }

    @Override
    public void process(DynamicTile tile) {
        tile.addCollider(colliderId, collider);
    }

}
