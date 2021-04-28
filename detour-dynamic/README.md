## Dynamic Nav Mesh

Dynamic nav mesh tries to address run-time modifications due to additional objects being added and removed.
The process starts with the voxelization of the initial static geometry which is then serialized to a file. 
The serialized voxel data serves as a basis for the nav mesh creation process.
Once the desired colliders (additional objects) are added or removed the nav mesh must be rebuilt.
Such colliders can both block the existing passage or create new passages (e.g. placing a bridge).
It is highly recommended to use tiled heightfields and multiple threads to rebuild dynamic nav meshes in order to reduce the build time.
A detailed example of how the to use dynamic nav meshes can be found in [DynamicNavMeshTest](https://github.com/ppiastucki/recast4j/blob/master/detour-dynamic/src/test/java/org/recast4j/dynamic/DynamicNavMeshTest.java#L32)

Supported Colliders:
* Sphere
* Capsule
* Box
* Cylinder
* Convex Trimesh
* Trimesh
* Composite (contains multiple colliders of various types)

### Comparison to Detour Tile Cache

Detour Tile Cache stores pre-processed voxel data (so-called compact heightfield) in order to improve the performance and memory utilization at the expense of flexibility and robustness. Simple obstacles which are supported by tile cache can only block existing passages and do not cause new paths to be created.

| | Dynamic Nav Mesh | Detour Tile Cache |
| --- | --- | --- |
| Obstacles | Simple and complex (see supported colliders) | Very simple |
| Obstacles block passages | Yes | Yes |
| Obstacles create passages | Yes | No |
| Memory required | More | Less |
| Build performance | Worse | Better |
| Build configuration | Full | Partial |
