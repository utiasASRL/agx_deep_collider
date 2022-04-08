# Fork of CrowdNav

This project is forked from the CrowdNav simulator **[`Website`](https://www.epfl.ch/labs/vita/research/planning/crowd-robot-interaction/) | [`Paper`](https://arxiv.org/abs/1809.08835) | [`Video`](https://youtu.be/0sNVtQ9eqjA)**. The original project includes the ```crowd_sim``` package, an OpenAI gym-based crowd navigation simulator and the ```crowd_nav``` package, an RL-based crowd navigation algorithm. We have added the ```simple_nav``` package.

The main objective of our contribution is to explore manifestations of the \ac{FRP} by benchmarking state of the art algorithm associated with each of the three approaches. % above with respect to the Freezing Robot Problem (FRP). Prospective algorithms that we plan to compare include canonical methods from ROS NavStack's TEB and DWA as well as learning-based  based methods.


## CrowdNav (original)
The ReadMe for the original CrowdNav project can be found in the [`original GitHub repo`](https://github.com/vita-epfl/CrowdNav).

### Citation
This is the source to the original paper:
```bibtex
@inproceedings{chen2019crowd,
  title={Crowd-robot interaction: Crowd-aware robot navigation with attention-based deep reinforcement learning},
  author={Chen, Changan and Liu, Yuejiang and Kreiss, Sven and Alahi, Alexandre},
  booktitle={2019 International Conference on Robotics and Automation (ICRA)},
  pages={6015--6022},
  year={2019},
  organization={IEEE}
}
```
