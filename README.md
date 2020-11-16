# CooperativeScheduler
This project was developed by Marla Ebeid, Omar Helmy & Shireen Afify using Keil uVision. It is a cooperative scheduler written in C using the Renode simulator, it aims to schedule given tasks with respect to their priority (highest priority first) and it handles up to 8 priority levels. Any task can rerun itself and sleep for a specific period of time.We made a dictionary that maps each task to an id, for example, TaskA:1, TaskB:2...We implemented two applications to demonstrate our code: 

1- Have 4 tasks A,B,C,D, ReRunMe(13,4,1); ReRunMe(7,1,2);ReRunMe(4,2,3);ReRunMe(11,3,4); 

2- Have 2 Tasks only A and B with priorities 3 and 4 respectively ReRunMe(3,3,1) and ReRunMe(1,4,2);

These two applications were tested and worked correctly. 

How to run the code:

Download Renode and enter this command: s @scripts/single-node/stm32f4_discovery.resc , the scheduler will be displayed 
