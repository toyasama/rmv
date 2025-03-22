# RMV Project

## Description

**Ros Markers Visualization** is a project that aims to render ROS2 markers as an image using a **top-down view**.

---

## Table of Contents

1. [About](#about)
2. [Prerequisites](#prerequisites)
3. [Installation](#installation)

---

## About

During development, it is often useful to have a quick way to visualize data using ROS markers. However, using software like RViz is not always practical or even possible.

RMV allows you to define a configuration for the data you want to visualize and publish it as a ROS2-compatible image, making it easily integrable into your workflow.

---

## Prerequisites

- ROS 2
- CycloneDDS

*In order to use the desktop app*
- Docker
- Docker Compose

---

## Installation

1. Clone the repository:
   ```sh
   git clone https://github.com/toyasama/rmv.git

*In order to use it as a stand alone separtly from your workspace*
1. Launch docker
    ```sh
    docker-compose up
    ```

## How to use

1. In your workspace
- copy **library** and **rmv_chore** in your workspace.
- Build the workspace
- Configure the yaml **params.yml** as desired.
- Run code :
   ```sh
   ros2 run rmv_chore rmv_chore
   ```
2. In docker 
- Configure the yaml **params.yml** as desired.
...