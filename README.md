Recast4j
========

[![Build Status](https://travis-ci.org/ppiastucki/recast4j.svg?branch=master)](https://travis-ci.org/ppiastucki/recast4j)

Java Port of Recast and Detour navigation mesh toolset.

![screenshot of a navmesh baked with the sample program](/recast-demo/screenshot.png?raw=true)

## Recast

Recast is state of the art navigation mesh construction toolset for games.

* It is automatic, which means that you can throw any level geometry at it and you will get robust mesh out
* It is fast which means swift turnaround times for level designers
* It is open source so it comes with full source and you can customize it to your heart's content. 

The Recast process starts with constructing a voxel mold from a level geometry 
and then casting a navigation mesh over it. The process consists of three steps, 
building the voxel mold, partitioning the mold into simple regions, peeling off 
the regions as simple polygons.

1. The voxel mold is built from the input triangle mesh by rasterizing the triangles into a multi-layer heightfield. Some simple filters are  then applied to the mold to prune out locations where the character would not be able to move.
2. The walkable areas described by the mold are divided into simple overlayed 2D regions. The resulting regions have only one non-overlapping contour, which simplifies the final step of the process tremendously.
3. The navigation polygons are peeled off from the regions by first tracing the boundaries and then simplifying them. The resulting polygons are finally converted to convex polygons which makes them perfect for pathfinding and spatial reasoning about the level. 


## Detour

Recast is accompanied with Detour, path-finding and spatial reasoning toolkit. You can use any navigation mesh with Detour, but of course the data generated with Recast fits perfectly.

Detour offers simple static navigation mesh which is suitable for many simple cases, as well as tiled navigation mesh which allows you to plug in and out pieces of the mesh. The tiled mesh allows you to create systems where you stream new navigation data in and out as the player progresses the level, or you may regenerate tiles as the world changes. 

More information about [Recast and Detour](https://github.com/recastnavigation/recastnavigation)

## Java Version
### How To Use
The API is kept as close to https://github.com/recastnavigation/recastnavigation as possible so most of the information and hints apply to recast4j too.
You can find a lot of examples in tests e.g.
- building a nav mesh from obj files: https://github.com/ppiastucki/recast4j/blob/master/detour/src/test/java/org/recast4j/detour/RecastTestMeshBuilder.java
- finding a path: https://github.com/ppiastucki/recast4j/blob/master/detour/src/test/java/org/recast4j/detour/FindPathTest.java#L94
- persisting a nav mesh: https://github.com/ppiastucki/recast4j/blob/master/detour/src/test/java/org/recast4j/detour/io/MeshSetReaderWriterTest.java
### Recast Demo
### Java Version Enhancements
#### Recast
- out-of-the-box support for multi-threaded builds
#### Detour-tile-cache
- more compact file format due to reduced data structures and better compression with LZ4
#### Extras
- simple tool to import navmeshes created with [A* Pathfinding Project](https://arongranberg.com/astar/)

### Maven
#### Releases
Recast4j releases are available in Maven Central Repository.
The project includes 4 artifacts:
```
<dependency>
	<groupId>org.recast4j</groupId>
	<artifactId>recast</artifactId>
	<version>1.2.0</version>
</dependency>
<dependency>
	<groupId>org.recast4j</groupId>
	<artifactId>detour</artifactId>
	<version>1.2.0</version>
</dependency>
<dependency>
	<groupId>org.recast4j</groupId>
	<artifactId>detour-crowd</artifactId>
	<version>1.2.0</version>
</dependency>
<dependency>
	<groupId>org.recast4j</groupId>
	<artifactId>detour-tile-cache</artifactId>
	<version>1.2.0</version>
</dependency>
<dependency>
	<groupId>org.recast4j</groupId>
	<artifactId>detour-extras</artifactId>
	<version>1.2.0</version>
</dependency>
```

#### Snapshots
Recast4j snapshots are currently available in Sonatype snapshots repository.
```
<repositories>
	<repository>
		<id>snapshots-repo</id>
		<url>https://oss.sonatype.org/content/repositories/snapshots</url>
		<releases>
			<enabled>false</enabled>
		</releases>
		<snapshots>
			<enabled>true</enabled>
		</snapshots>
	</repository>
</repositories>
```

## License

Recast & Detour is licensed under ZLib license, see License.txt for more information.
