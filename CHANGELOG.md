# Changelog

## [0.3.0] - 2025-06-15

### <!-- 0 -->⛰️  Features

- Add obstacle avoidance and visualization

### <!-- 2 -->🚜 Refactor

- Refactor examples and remove dead code

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Refactor for updated concord point structure

## [0.2.1] - 2025-06-13

### <!-- 6 -->🧪 Testing

- Refine field geometry handling and testing

## [0.2.0] - 2025-06-12

### <!-- 0 -->⛰️  Features

- Use concord's polygon partitioner
- Docs: Explain and exemplify new partitioning
- Implement advanced multi-criteria field partitioning
- Add obstacle avoidance to Rerun visualization
- Implement basic obstacle avoidance functionality
- Generate and visualize connections between swaths
- Per-indivifual parts v3
- Per-indivifual parts v2
- Improve swath traversal and reordering logic
- Return swathsh instead of vertices
- Per-indivifual parts
- Use intial point automatically (still some segfault though)
- Initial swath return support for nety
- Refactor geometry constructors and simplify usage

### <!-- 1 -->🐛 Bug Fixes

- Stupid pointer issue with intil point for graph

### <!-- 2 -->🚜 Refactor

- Refactor Swath representation and handling

### <!-- 6 -->🧪 Testing

- Improve Geometry Utility Testing and Formatting

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Fix gitignore entry for compile commands
- Ignore compile_commands.json

### Build

- Refactor build system and code style

## [0.1.2] - 2025-05-30

### <!-- 0 -->⛰️  Features

- Implement agricultural field traversal networks utilizing AB lines
- Add swath tour visualization with connections
- Optimize swath path calculation with 2-opt local search
- Feat: Introduce graph algorithm to example
- Refactor and implement new division strategies
- Update fleet visualization to use Divy object
- Feat: Add support for field division visualization
- Feat: Add machine division visualization
- Visualize field swaths in example
- Refine field generation and related plotting
- Refactor geometry processing for field generation
- Refactor border polygon and add swath count to example
- Introduce noise and geometry for field generation
- Convert to geotiff and FarmTrax data structures
- Update geojson with new coordinate data
- Add GeoJSON example code and dependencies
- Incorporate concord for codebase linking
- Init

### <!-- 1 -->🐛 Bug Fixes

- Adjust polygon handling and datum

### <!-- 2 -->🚜 Refactor

- Introduce partitioning for field division
- Introduce R-trees for spatial indexing of geometry
- Rename and refactor build scripts and aliases

### <!-- 3 -->📚 Documentation

- Document the Graph + R-tree architecture

### <!-- 4 -->⚡ Performance

- Improve tracing algorithm with focus on field patterns

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Refactor release script and visualization utilities
- Update build and add C++ example


