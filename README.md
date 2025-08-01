# Key Interval A*


## 📋 Table of Contents

- [Overview](#overview)
- [Algorithm Description](#algorithm-description)
- [Key Features](#key-features)
- [Installation](#installation)
- [Usage](#usage)
- [Experimental Results](#experimental-results)
- [File Structure](#file-structure)
- [Dependencies](#dependencies)
- [Citation](#citation)
- [License](#license)

## 🎯 Overview
Key-Interval A* introduces a higher-level abstraction based on intervals — maximal traversable segments
on grid rows or columns — and extracts key points that capture structural changes in interval boundaries through horizontal and vertical scans. By restricting search to these key intervals and navigating via key points, the algorithm reduces the number of node expansions and significantly improves the search speed.


## 🔬 Algorithm Description

### Core Components

1. **Preprocessing Module** (`src/KIAstar/Preprocess.*`)
   - Automatically identifies key intervals in the environment
   - Computes connectivity between intervals
   - Builds efficient data structures for fast querying

2. **Key Interval A* Search** (`src/KIAstar/KeyIntervalAStar.*`)
   - Implements the main search algorithm
   - Uses key intervals to guide exploration
   - Maintains optimality through careful cost calculations

3. **Benchmarking Framework** (`benchmark/`)
   - Comprehensive evaluation against baseline algorithms
   - Performance metrics collection and analysis
   - Statistical significance testing


## 🚀 Installation

### Prerequisites

- CMake 3.14 or higher
- C++20 compatible compiler (GCC 10+, Clang 12+, or MSVC 2019+)
- Git


## 📖 Usage

### Basic Usage

```cpp
#include "src/KIAstar/KeyIntervalAStar.h"
#include "src/KIAstar/Preprocess.h"

// Create preprocessing object
Preprocess preprocess;

// Load grid map
std::vector<std::vector<int>> grid = loadGridMap("map.txt");

// Preprocess the environment
preprocess.preprocess(grid);

// Create KIA* solver
KeyIntervalAStar solver(preprocess);

// Define start and target positions
Vertex start(0, 0);
Vertex target(100, 100);

// Find path
std::vector<Vertex> path = solver.search(start, target);
```

### Running Benchmarks

```bash
# Run single test
./bin/single_test

# Run comprehensive benchmark
./bin/benchmark


```


## 📁 File Structure

```
├── src/
│   ├── KIAstar/              # Core KIA* implementation
│   │   ├── KeyIntervalAStar.cpp
│   │   ├── KeyIntervalAStar.h
│   │   ├── Preprocess.cpp
│   │   └── Preprocess.h
│   ├── astar/                # Baseline A* implementation
│   ├── jps/                  # Jump Point Search implementation
│   ├── hpastar/              # HPA* implementation
│   └── utilities/            # Utility functions
├── benchmark/                # Benchmarking framework
│   ├── Benchmark.cpp
│   ├── BenchmarkUtils.cpp
│   └── SingleTest.cpp
├── test/                     # Unit tests
├── data/                     # Test data and maps
├── data_loader/              # Data loading utilities
├── CMakeLists.txt            # Build configuration
└── README.md                 # This file
```

## 🔧 Dependencies

- **CMake**: Build system
- **Google Test**: Unit testing framework (automatically downloaded)
- **C++20 Standard Library**: Modern C++ features


## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
