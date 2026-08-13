# CS452 Trains: Kernel and Train Controller Program

This repository contains a kernel for the Raspberry Pi 4b, as well as a train controller program that runs on the kernel. The kernel was written from scratch, including but not limited to the following features:
- Independant tasks, including all context switching, scheduling
- IPC communication using send / receive / reply protocol
- Interrupts (GPIO, UART, CAN, Timer)
- Drivers for UART and CAN (mcp2515, through SPI)
- Separation of Kernel & User-level code through exception levels and system calls 

The train controller program is a user-level program that runs on the kernel and controls a model train set. It uses the IPC communication to split responsibilities between multiple tasks in a MVC fashion. The trains are programmed using behaviour trees, with each train having it's own task & behaviour tree. If you're not familiar with behaviour trees, I suggest reading up on them [here](https://wboayue.com/posts/behavior-trees/) and [here](https://www.behaviortree.dev/docs/learn-the-basics/BT_basics), as they are widely used for programming AI in video games, drones, and even Roombas. 
The train program allows upwards of 4 trains to run smoothly on the track, with deadlock avoidance / recovery, track reservations, and accurate modelling. Note that the trains do not expose any "speed" or "location" parameters, and train location is modelled in the train control program. 

There is an accompanying [web visualizer](https://ant52ho.github.io/#/track-map) built for the train controller program that communicates with the Raspberry pi through the secondary UART interface  to visualze the train's paths and track reservations.