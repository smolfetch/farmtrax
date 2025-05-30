
<img align="right" width="26%" src="./misc/logo.png">

Farmtrax
===

A C++ header-only library for generating AB-lines and optimizing field traversal paths for agricultural robots and tractors.

---

## Overview

Farmtrax provides sophisticated algorithms for:
- **Field Processing**: Convert GeoJSON polygons into workable farm fields with AB-lines (work paths)
- **Spatial Partitioning**: Divide fields among multiple machines for efficient parallel work
- **Path Optimization**: Generate optimal traversal sequences using graph theory and spatial indexing
- **Agricultural Patterns**: Respect real-world farming practices and field geometry

## Key Features

- 🚜 **Agricultural AB-Line Generation**: Create work paths with A and B endpoints
- 🗺️ **Multi-Machine Division**: Intelligent field partitioning for fleet operations  
- 🌾 **Field-Aware Traversal**: Graph-based path optimization with spatial clustering
- 📊 **R-tree Spatial Indexing**: Efficient nearest-neighbor queries for large fields
- 📈 **Real-time Visualization**: Integration with Rerun for 3D field visualization
- ⚡ **High Performance**: Optimized algorithms for real-time agricultural applications

---

## Installation

### CMake

```cmake
FetchContent_Declare(
  farmtrax
  GIT_REPOSITORY https://github.com/bresilla/farmtrax.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(farmtrax)

target_link_libraries(<lib/bin> PRIVATE farmtrax::farmtrax)
```

### Dependencies

- **Boost Geometry**: For spatial operations and R-tree indexing
- **Boost Graph**: For graph algorithms (Dijkstra, traversal)
- **Concord**: Coordinate system transformations
- **GeoSON**: GeoJSON parsing and writing
- **Rerun** (optional): For 3D visualization

---

## Quick Start

### Basic Usage

```cpp
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp"
#include "farmtrax/divy.hpp"

// 1. Load field from GeoJSON
concord::Polygon poly;
auto fc = geoson::ReadFeatureCollection("field.geojson");
poly = std::get<concord::Polygon>(fc.features[0].geometry);

// 2. Create field with AB-lines
concord::Datum datum{51.989, 5.658, 53.801};
farmtrax::Field field(poly, 0.1, datum, true, 0.5);
field.gen_field(4.0, 0.0, 3); // 4m spacing, 0° angle, 3 parts

// 3. Divide field among machines
auto fieldPtr = std::make_shared<farmtrax::Field>(field);
farmtrax::Divy divy(fieldPtr, farmtrax::DivisionType::ALTERNATE, 4);

// 4. Generate optimal paths for each machine
auto& res = divy.result();
for (size_t m = 0; m < 4; ++m) {
    // Convert swaths to AB lines
    std::vector<std::pair<concord::Point, concord::Point>> ab_pairs;
    for (const auto& swath : res.swaths_per_machine.at(m)) {
        ab_pairs.emplace_back(swath->line.getStart(), swath->line.getEnd());
    }
    
    // Create graph-based path optimizer
    farmtrax::Nety nety(ab_pairs);
    
    // Generate optimal traversal path
    auto path = nety.field_traversal(ab_pairs[0].first);
    
    std::cout << "Machine " << m << " path: " << path.size() << " vertices\n";
}
```

---

## Core Components

### 1. Field Generation (`farmtrax::Field`)

Converts polygon boundaries into workable farm fields:

```cpp
farmtrax::Field field(polygon, resolution, datum, add_headlands, headland_width);
field.gen_field(spacing, angle, num_parts);
```

- **Polygon Processing**: Handles complex field boundaries with holes
- **AB-Line Generation**: Creates parallel work paths across the field
- **Headland Management**: Automatic headland generation for turning areas
- **Multi-Part Support**: Divides large fields into manageable sections

### 2. Machine Division (`farmtrax::Divy`)

Intelligently partitions work among multiple machines:

```cpp
farmtrax::Divy divy(field_ptr, farmtrax::DivisionType::ALTERNATE, num_machines);
```

**Division Types:**
- `ALTERNATE`: Alternating swath assignment for balanced workload
- `BLOCK`: Contiguous blocks for minimal inter-machine conflicts
- `CUSTOM`: User-defined division strategies

### 3. Path Optimization (`farmtrax::Nety`)

Advanced graph-based traversal optimization using spatial algorithms.

---

## Graph + R-tree Architecture

The core innovation of Farmtrax is the **Nety** class, which combines graph theory with spatial indexing for optimal field traversal.

### Architecture Overview

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   AB Lines      │───▶│   Nety Class     │───▶│ Optimal Path    │
│ (Work Paths)    │    │                  │    │ (Vertex List)   │
└─────────────────┘    │ ┌──────────────┐ │    └─────────────────┘
                       │ │  R-tree      │ │
                       │ │  Spatial     │ │
                       │ │  Index       │ │
                       │ └──────────────┘ │
                       │ ┌──────────────┐ │
                       │ │  Boost       │ │
                       │ │  Graph       │ │
                       │ └──────────────┘ │
                       └──────────────────┘
```

### R-tree Spatial Indexing

The R-tree provides **O(log n)** spatial queries for efficient nearest-neighbor searches:

```cpp
using EndpointTree = boost::geometry::index::rtree<PointRTreeValue, 
                     boost::geometry::index::quadratic<16>>;
```

**Key Benefits:**
- **Fast Spatial Queries**: Find nearby endpoints in logarithmic time
- **Progressive Search**: Expanding radius search (15m → 30m → 50m → 100m → 200m → 500m)
- **Spatial Clustering**: Groups nearby parallel lines to minimize cross-field jumps

### Graph Structure

Each AB line creates two vertices (A and B endpoints) connected by edges:

```cpp
// Graph components
using Graph = boost::adjacency_list<boost::listS, boost::vecS, 
                                   boost::undirectedS, VertexProps, EdgeProps>;
std::vector<Vertex> vertex_A_; // A endpoints for each line  
std::vector<Vertex> vertex_B_; // B endpoints for each line
```

**Edge Types:**
1. **Work Edges**: A↔B connections (actual field work)
2. **Transition Edges**: Connections between different AB lines (movement)

### Field-Aware Traversal Algorithm

The traversal algorithm combines multiple heuristics:

#### 1. Progressive Spatial Search
```cpp
const std::vector<double> search_radii = {15.0, 30.0, 50.0, 100.0, 200.0, 500.0};
```
- Starts with tight 15m radius for local clustering
- Expands progressively if no suitable candidates found
- Prevents excessive cross-field connections

#### 2. Agricultural Pattern Scoring
```cpp
double score = distance + direction_penalty - pattern_bonus;
```

**Distance Component**: Base cost (shorter is better)
**Direction Penalty**: Prefers perpendicular movement to line orientation
**Pattern Bonus**: Rewards maintaining parallel line clusters

#### 3. Orientation-Aware Connections
```cpp
bool lines_have_similar_orientation(size_t line1_id, size_t line2_id) const {
    // Check if lines are parallel within 15° tolerance
    return angle < (15.0 * M_PI / 180.0);
}
```

### Connection Quality Control

Smart connection management prevents unrealistic paths:

- **Distance Threshold**: Maximum 150m connection radius
- **Connection Limit**: At most 4 connections per endpoint  
- **Parallel Preference**: 50% penalty for non-parallel connections
- **Clustering Bonus**: Rewards local field patterns

### Performance Characteristics

| Operation | Time Complexity | Description |
|-----------|----------------|-------------|
| R-tree Insert | O(log n) | Adding endpoints to spatial index |
| Nearest Neighbor | O(log n) | Finding closest unvisited endpoints |
| Graph Construction | O(n log n) | Building connectivity graph |
| Dijkstra Path | O((V + E) log V) | Shortest path calculation |
| Field Traversal | O(n² log n) | Complete field traversal |

### Real-World Benefits

1. **Reduced Travel Time**: Minimizes non-productive movement between work areas
2. **Fuel Efficiency**: Shorter transition paths reduce fuel consumption  
3. **Field Preservation**: Avoids excessive cross-field traffic
4. **Scalable Performance**: Handles large fields with hundreds of AB lines
5. **Agricultural Compliance**: Respects real farming practices and patterns

---

## Examples

### Complete Field Processing Pipeline

```cpp
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp" 
#include "farmtrax/divy.hpp"
#include "farmtrax/utils/visualize.hpp"

int main() {
    // Load and process field
    auto fc = geoson::ReadFeatureCollection("field.geojson");
    concord::Polygon poly = std::get<concord::Polygon>(fc.features[0].geometry);
    
    concord::Datum datum{51.989, 5.658, 53.801};
    farmtrax::Field field(poly, 0.1, datum, true, 0.5);
    field.gen_field(4.0, 0.0, 3);
    
    // Multi-machine division
    auto fieldPtr = std::make_shared<farmtrax::Field>(field);
    farmtrax::Divy divy(fieldPtr, farmtrax::DivisionType::ALTERNATE, 4);
    
    // Process each machine
    auto& res = divy.result();
    for (size_t m = 0; m < 4; ++m) {
        if (res.swaths_per_machine.at(m).empty()) continue;
        
        // Convert to AB lines
        std::vector<std::pair<concord::Point, concord::Point>> ab_pairs;
        for (const auto& swath : res.swaths_per_machine.at(m)) {
            ab_pairs.emplace_back(swath->line.getStart(), swath->line.getEnd());
        }
        
        // Optimize traversal path  
        farmtrax::Nety nety(ab_pairs);
        auto start_point = ab_pairs[0].first;
        auto path = nety.field_traversal(start_point);
        
        std::cout << "Machine " << m << ": " << path.size() << " vertices, "
                  << nety.calculate_path_distance(path) << "m total distance\n";
    }
    
    return 0;
}
```

### Custom AB Line Processing

```cpp
// Manual AB line creation
std::vector<farmtrax::ABLine> custom_lines;
custom_lines.emplace_back(
    farmtrax::BPoint(0, 0),    // A endpoint
    farmtrax::BPoint(100, 0),  // B endpoint  
    0                          // Line ID
);

farmtrax::Nety nety(custom_lines);
auto path = nety.field_traversal(farmtrax::BPoint(0, 0));
```

---

## Build Instructions

### Prerequisites

```bash
# Ubuntu/Debian
sudo apt install cmake build-essential libboost-all-dev

# macOS  
brew install cmake boost

# Or use devbox (recommended)
devbox shell
```

### Building

```bash
# Using provided build script
./run.sh b    # Build
./run.sh r    # Run example

# Manual CMake
mkdir build && cd build
cmake ..
make -j$(nproc)
./main
```

---

## Visualization

Farmtrax integrates with [Rerun](https://rerun.io/) for real-time 3D visualization:

```cpp
auto rec = std::make_shared<rerun::RecordingStream>("farmtrax", "space");
rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy");

farmtrax::visualize::show_field(field, rec);
farmtrax::visualize::show_swath_tour(nety, path, rec, machine_id);
```

**Visualization Features:**
- 3D field boundaries and topography
- AB line visualization with directional arrows
- Machine path visualization with color coding
- Real-time path optimization updates
- Connection quality indicators

---

## Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---
