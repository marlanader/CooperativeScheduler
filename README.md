# CooperativeScheduler
This project was developed by Marla Ebeid, Omar Helmy & Shireen Afify using Keil uVision. It is a cooperative scheduler written in C using the Renode simulator, it aims to schedule given tasks with respect to their priority (highest priority first) and it handles up to 8 priority levels.The code starts by Calling QueTask function in the main and passing to it the Task,priority and id of the task you want to implement. The tasks will then be inserted into the ready queue and sorted based on their priorities, then the dispatch function should be called in the main (while loop) that dispatches the highest priority task. Any task can rerun itself by calling the ReRun Function and passing to it the sleep time, task priority and task id. If the sleep time is 0 the task will directly be sent to the ready queue, else it will be sent to the delayed queue (using QueDelayTask function) that has all the delayed tasks sorted based on their sleep time (that is decremented by 1 each tick, each tick is 100msec and this is handled by systick in the systick_handler function). We made a dictionary that maps each task to an id, for example, TaskA:1, TaskB:2, TaskC:3...till TaskH:8. We used Uart2 for sending messages to renode.

We implemented two applications to demonstrate our code: 

1- Have 4 tasks A,B,C,D, ReRunMe(13,4,1); ReRunMe(7,1,2);ReRunMe(4,2,3);ReRunMe(11,3,4); (these function calls should be called in each of the Tasks function)

2- Have 2 Tasks only A and B with priorities 3 and 4 respectively ReRunMe(3,3,1) and ReRunMe(1,4,2);

These two applications were tested and worked correctly. 

How to run the code:

Download keil uVision and create a new project then add the main.c file to it and then run it
Download Renode (search for the path of the stm32f4_discovery.resc) and enter this command: s @scripts/single-node/stm32f4_discovery.resc , the scheduler will be displayed 
