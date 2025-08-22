# BluRaspberry
Optimal Autonomous Underwater Vehicles(AUVs) Path Planning

**Folder Structure**
* `final_notebooks` Notebooks that you can check out if you interested in the process or reproduce what we did.
* `data` Dataset are shared here.

<!--More elaborate structure for a data science project: [Here](https://cookiecutter-data-science.drivendata.org/#directory-structure) is an example of a more elaborate structure for a data science project.-->

## One-line Description
We aim to develop a system that determine the real-time optimal path for an AUV to explore biologically active regions across a transect of North Atlantic.

## Collaborators

| Name                | Role                |
|---------------------|---------------------|
| Erin Linebarger     | Participant         |
| Joyce Cai           | Participant         |
| Kayla Mitchell      | Participant         |
| Christian Sarason   | Project Facilitator |

## Planning

* Initial idea: use numerical model outputs to simulate potential scenarios for the UUV
* Ideation jam board: [link](https://www.figma.com/board/mTpn6HdqrURccqMxZJdo3D/blue-raspberry?node-id=0-1&t=hRSpES27tXPrWp6H-1)
* Slack channel: #ohw25_proj_blu_raspberry
* Final presentation: Add link

## Background

## Goals
**Optimization Function for Path Planning**
- Develop an algorithm that dynamically adjusts AUV navigation points to maximize sampling efficiency of biological targets (e.g., algal blooms).
- Incorporate environmental variables such as currents, temperature, salinity, oxygen utilization, water age, and seafloor distribution.

**Integration with Robotic Control Systems**
- Build scripts to interface between Python-based optimization functions and ROS2, potential enabling direct deployment on AUV platforms.
- Ensure compatibility with Gazebo for simulation testing.

**Scalable Cloud Implementation**
- Run scripts on large oceanographic datasets in cloud environments .


## Datasets and Methodology
**AUV Control and Simulation**
- ROS2
- Gazebo

**Simulated Ocean field**
- OCIM (Ocean Circulation Inverse Model): A global ocean circulation inverse model that estimates large-scale transport and tracer distributions using observational constraints. We use 2° × 2° horizonal resolution and 24 layer.
- HYCOM: A high-resolution ocean circulation model that simulates three-dimensional ocean dynamics by blending isopycnal, terrain-following, and z-level coordinates. We use xxx resolution in the upper 1000 m.

**Region of interest and time range**
- The Gulf Stream in North Atlantic, (lat, lon)
- xx,xx-xx, 2025

**Score function**
develop the score function based on different considerations, including
- efficiency: utilize ocean current velocity (u,v,w)
- biological insteret: Gradient of Apparent Oxygen Utilization (AOU). Larger gradient, higer score. AOU is calculated based on temperature and salinity.


## Workflow/Roadmap

## Results/Findings

## Lessons Learned

## References

