## 🚀 Isaac Sim + Isaac Lab Reinforcement Learning Project

This project demonstrates how to set up and run a reinforcement learning (RL) workflow using NVIDIA Isaac Sim and Isaac Lab. The goal is to train a simulated quadruped “ant” robot to learn locomotion through trial and error, without explicitly programming movement rules.

Using Isaac Sim’s high-fidelity physics engine and Isaac Lab’s RL framework, multiple instances of the robot are simulated in parallel to accelerate training. A neural network policy is trained using the PPO (Proximal Policy Optimization) algorithm from the RSL-RL library. The agent observes its state (e.g., position, velocity, joint angles) and outputs joint torques, receiving rewards for moving forward, staying upright, and minimizing energy use.

The project includes:

* Installation and configuration of Isaac Sim (standalone) and Isaac Lab
* Linking both systems using the provided Python environment
* Running RL training via `train.py` with customizable parameters
* Monitoring learning progress through logs and live simulation

By the end of training, the robot learns stable and efficient walking behavior. This project serves as a hands-on introduction to modern robotics simulation, reinforcement learning, and scalable training workflows used in real-world AI and robotics research.
