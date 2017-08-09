Recast4j
========

[![Build Status](https://travis-ci.org/ppiastucki/recast4j.svg?branch=master)](https://travis-ci.org/ppiastucki/recast4j)

Java Port of Recast and Detour navigation mesh toolset.

More information about [Recast and Detour](https://github.com/recastnavigation/recastnavigation)

## License

Recast & Detour is licensed under ZLib license, see License.txt for more information.

## Usage
### Java Version Enhancements
#### Recast
- out-of-the-box support for multi-threaded build
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
	<version>1.0.6</version>
</dependency>
<dependency>
	<groupId>org.recast4j</groupId>
	<artifactId>detour</artifactId>
	<version>1.0.6</version>
</dependency>
<dependency>
	<groupId>org.recast4j</groupId>
	<artifactId>detour-crowd</artifactId>
	<version>1.0.6</version>
</dependency>
<dependency>
	<groupId>org.recast4j</groupId>
	<artifactId>detour-tile-cache</artifactId>
	<version>1.0.6</version>
</dependency>
<dependency>
	<groupId>org.recast4j</groupId>
	<artifactId>detour-extras</artifactId>
	<version>1.0.6</version>
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

