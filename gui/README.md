# 🚀 MockaFlow (Working Title)

**MockaFlow** is an ambitious, open-source simulator built around a node-based UI, inspired by systems like Simulink—with a few bold innovations of its own:

## 🔧 Key Features

- **Configurable & Flexible Nodes**  
  Each node can execute binaries written in a variety of programming languages.

- **ROS2 Integration (Phase 1)**  
  Initially designed to run ROS2 nodes behind the scenes, leveraging the full power of robotics middleware.

- **Middleware Agnostic (Future Vision)**  
  The architecture will support switchable middleware engines, offering complete flexibility for diverse use cases.

## 🖥️ Tech Stack

### Frontend
- **React** + **ReactFlow** for an intuitive and highly interactive node-based interface.
- **Vite** for DevServer and Production Server Bundling

### Backend
- **Python** with a lightweight framework like **Flask**, enabling seamless integration with ROS2.
- **C++** (via **pybind11**) for performance-critical computations, exposed to Python for maximum efficiency.

## 🌐 Project Goals

MockaFlow is designed to empower developers, researchers, and roboticists with a modular, adaptable simulation platform. Whether you're building complex systems or rapid prototypes, **MockaFlow puts flexibility first.**

## Usage

To run the development server locally:

`bazel run //gui:start`

Then, open your browser and navigate to:

http://localhost:5173/

## 🛠️ Developer Tips

### 🔄 Auto-reloading with ibazel
It's recommended to use ibazel ([bazel-watcher](https://github.com/bazelbuild/bazel-watcher?tab=readme-ov-file#installation)), a Bazel watcher that automatically restarts the server when source files change. This gives you instant feedback during development.  to automatically restart the bazel command whenever a src file is changed and immediately look at the updated webpage.

`ibazel run //gui:start`

### 📦 Managing JavaScript Dependencies
When modifying package.json—whether you're adding or updating libraries—run:

`./gui/tools/pnpm install`

This ensures the [pnpm-lock.yaml](/pnpm-lock.yaml) file stays up to date for Bazel to correctly fetch dependencies.

### 🧹 Troubleshooting Bazel Cache Issues
Sometimes Bazel's cache may cause persistent issues with JavaScript libraries. If errors keep appearing despite a correct setup, try clearing the cache:

`bazel clean --expunge`

This resets the Bazel cache for a fresh build environment.

## RoadMap

## Changelog
