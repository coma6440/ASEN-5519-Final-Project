## TODO
- <s>Adjust goal in .yaml files to be the true goal</s>
- <s> Move plotting to MATLAB due to z-ordering issue in python. Can use convhull in MATLAB </s>
- <s>Add goal regions to plots</s>
- <s> Write psuedocode for informed SST </s>
- <s> Figure out why planners are moving through obstacles. Seems to be related to fcl and having an obstacle in the middle? Objects are rotated. </s>
- <s> Implement kinodynamic benchmarking. See https://ompl.kavrakilab.org/benchmark.html </s>
- Can try restricting state space on the planner level in main.cpp
- <s> Define final environments to use for planning. Can try creating maze like structure. Use simple block and w6 </s>
- <s> Collect geometric results, used RRTconnect </s>
- <s> Start overleaf document </s>
- Collect results for current optimization objective. This includes cost
- Try different optimization objective? Time would be a good one to use since the robot can rotate when stopped.
- Add check for only updating path if future expected cost is better than current cost
