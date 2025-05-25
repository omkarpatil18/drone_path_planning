# Drone path planning
**Hardware-Software Co-Design for Path Planning by Drones**: https://ieeexplore.ieee.org/document/10802753

This work consists of two main components: designing a hardware-software co-design, MT+, for adapting the Mikami-Tabuchi algorithm for on-board path planning by drones in a 3D environment; and development of a specialized custom hardware accelerator CDU, as a part of MT+, for parallel collision detection. Collision detection is a performance bottleneck in path planning. MT+ reduces the delay in path planning without using any heuristic. A comparative analysis between the state-of-the-art path planning algorithm A* and Mikami-Tabuchi is performed to show that Mikami-Tabuchi is faster than A* in typical real-world environments. Mikami-Tabuchi is also preferred where an added requirement is a rectilinear path with minimum bends/turns. In custom-generated environments, path planning using Mikami-Tabuchi shows a latency improvement of  1.7x across varying average sizes of obstacles and 2.7x across varying obstacle density over state-of-the-art path planning algorithm, A*. Further, the experiments show that the co-design achieves speedups over a full software implementation on CPU, averaging between 10% to 60% across different densities and sizes of obstacles. CDU area and power overheads are negligible against a conventional single-core processor.



![Screenshot from 2024-03-15 21-54-56](https://github.com/user-attachments/assets/64f2d1b1-8b33-4ba8-9dbe-7b20610cbd54)


## Airgen Credits
```
@techreport{vemprala2023grid,
      title={GRID: A Platform for General Robot Intelligence Development}, 
      author={Sai Vemprala and Shuhang Chen and Abhinav Shukla and Dinesh Narayanan and Ashish Kapoor},
      year={2023},
      eprint={2310.00887},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
