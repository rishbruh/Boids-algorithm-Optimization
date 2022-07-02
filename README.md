# Boids-algorithm-Optimization

This project describes an implementation of the Boids algorithm by simulating a flock behaviour in artificial agents (boids). 
The clustering algorithms, DBSCAN and HDBSCAN are implemented in a Python environment and provide us with an improved performance of the boids algorithm. The performance is estimated using the average FPS of our simulations over a specified time period. While the DBSCAN algorithm gives a slight improvement over the original boids algorithm, the HDBSCAN algorithm achieves a drastically better performance than the original boids algorithm. The results of the simulations are statistically verified by conducting a one-way ANOVA test to detect any significant difference between the algorithms and a post-hoc test is further conducted to compare the effect between the original algorithm and the two clustering algorithms. 

The use of HDBSCAN algorithm for simulating the flock behaviour in the boids, gives rise to a novel emergent behaviour in the flock simulations. This new emergent behaviour is characterized and observed to understand its nature of occurrence.


![image](https://user-images.githubusercontent.com/62597096/177004278-95add633-afec-49d5-914e-756f14c21e08.png)

Step by step display of the new emergent behaviour - “disoriented-scattering behaviour”

Refer to the attached document - "Optimizing Boids algorithm using clustering algorithms.pdf" for the details of this study.

