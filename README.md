# Where is the Math in Path Planning for Autonomous Vehicles (Spline Smoothing) by Jorge González Zavala

## Introduction
Path planning is a crucial component in autonomous vehicle navigation. It helps tell the vehicle where and how to move. Not just to reach a destination, but to do so safely, efficiently, and smoothly. Every time I’m stuck in traffic, I can’t help but imagine how much better things would be if cars were driven in a more coordinated and intelligent way. Not by humans who can get tired or distracted, but by machines that can react quickly and precisely, keeping mobility organized and predictable.

My name is **Jorge González Zavala**, and I am a Mechanical Systems Engineering student at the **Arkansas State University Campus Querétaro**. I have a deep interest in emerging technologies, especially those that aim to fix long-standing inefficiencies in daily life. One of those problems that frustrates me almost daily is traffic, especially in my city, where public mobility is limited and poorly planned. It makes it hard not to think, *there must be a better way*.

In this project, I explored some foundational math behind path smoothing for autonomous vehicles. Specifically, I implemented an example of path smoothing using spline-based optimization in Python. This is just one small part of how a self-driving car figures out how to reach its destination. While initial path planners often generate a route that avoids obstacles and reaches the goal, those paths are not always smooth or even drivable by a real vehicle. A viable path must also respect limits on curvature, acceleration, and directional change. In other words, it needs to be smooth, continuous, and physically realistic. And that is where math comes in.

<div align="center">
  <img src="https://www.mathworks.com/help/examples/driving/win64/SmoothAPlannedPathExample_03.png" alt="Example of path smoothing from a rough planned trajectory" width="600"/><br>
  <em>Image source: <a href="https://www.mathworks.com/help/driving/ug/smooth-a-planned-path.html">MathWorks – Smooth a Planned Path</a></em>
</div>


### Updated Project Proposal

For this project, I chose to explore a real-world problem tied to the growing presence of self-driving cars. Specifically, how these vehicles determine the path to follow when trying to reach a destination. This problem is known as **path planning**.

Autonomous driving remains a controversial topic. Not as much as in past years, I must say. There are several instances of its implementation for delivery, public transportation, and personal mobility already. Still, while some people view it as a solution to traffic inefficiencies and accident reduction, others remain skeptical about its risks, ethics, and potential failures. However, regardless of public opinion, autonomous vehicles are becoming more common. So, making sure their decision-making systems are safe and reliable is more important than ever.

While I am not particularly passionate about cars or racing, like my brother is, for example, I've always been interested in **technological progress**. He and I have had debates ranging from whether electronic cars are a "real" solution to whether speed matters more than practicality. These debates often lead us to one topic we both find fascinating: **self-driving vehicles**.  My brother has his doubts, but I believe that, when well-designed and implemented, autonomous systems have the potential to significantly reduce traffic problems and improve road safety. Nevertheless, to make that happen, one of the fundamental components that must be perfected and ensured to work properly is path planning.

That is why I decided to focus on learning how the **optimization of the trajectory** an autonomous vehicle would follow is fundamentally implemented. More specifically, I explored a 2-step process:
1. A rough planner (such as **RRT**) is used to generate an initial path that helps get from point A to point B, following road constraints and avoiding obstacles.
2. **Spline-based optimization** is used to smooth that path, reducing sharp turns, high curvature, and unrealistic transitions for the vehicle to traverse the road comfortably.
I was particularly interested in learning to implement this using Python and numerical optimization tools.

Academically, I have completed math courses, such as Calculus I-III, Differential Equations, Linear Algebra, and Numerical Methods. This project gave me the chance to apply those skills and understand a real and relevant engineering problem. Furthermore, since I focused my work on the **numerical implementation** of a cost-based optimization model, I got the chance to explore both the theory and the computational side of the problem.

<div align="center">
  <a href="https://www.youtube.com/watch?v=PSX18U1fYEY&t=250s" target="_blank"><img src="https://i.ytimg.com/vi/PSX18U1fYEY/maxresdefault.jpg" alt="Self-Driving Cars - Lecture 12.4 (Decision Making and Planning: Motion Planning)" width="480" height="360"/></a>
  <p><em>This video provides a clear basic explanation of how path planning optimization works (Recommended segment: 4:10 - 9:10).</em></p>
</div>
<br>

## Hands-On: Path Smoothing via Spline Optimization\

### Scope
This project is focused entirely on **2D path planning**. Both the planner and the smoothing algorithms assume a flat plane with no changes in elevation. While real-world autonomous vehicles operate in a quasi-3D environment and consider orientation, velocity, and acceleration constraints, this implementation is limited to geometric pathfinding in a 2-dimensional space. The goal is to understand and demonstrate the mathematical foundation of planning and optimization before expanding into more complex models.

### From Random Trees to Drivable Paths
Before the smoothing process, a path to smooth is needed. For this example, the **RRT\* (Rapidly-Exploring Random Tree Star)** algorithm will be used.

Other classic shortest-path algorithms like **Dijkstra** or **A\*** work on a predefined grid of possible paths. In contrast, RRT and its variants are widely used in motion planning because of their ability to quickly explore large, continuous spaces and handle complex obstacles without a full grid map.

While **RRT** is fast and simple, it has a significant drawback:
> It stops as soon as it reaches the goal. Even if the path is long or inefficient.

RRT*, on the other hand, continues sampling even after finding a path, constantly trying to improve the existing connections. As the tree is "rewired", **shorter, more efficient paths** are found over time.

RRT* follows the following steps:
1. **Initialization**. It starts the *tree* with a *root node*. In this case, the vehicle's current position.
> Each node will store:
> - Its coordinates
> - A pointer to its parent node
> - The accumulated `cost()` from the root node (usually distance)
2. **Sample Point**. A random point (`x_rand`) is generated within the *configuration space*. Occasionally, this new point is replaced with the *destination node* to **bias the tree toward the destination**.
> The configuration space is the clear drivable space, avoiding obstacles and keeping within road constraints
3. **Nearest Node**. The closest previously *existing node* to `x_rand` is found.
4. **Steering Toward Random Point**. Generate a new node (`x_new`) stepping from the nearest node toward `x_rand` using a fixed step size. Check that the connection is collision-free. `x_new` is initially steered from the nearest node, but the parent is not yet final. A better parent may be selected in the next step.
5. **Node Neighborhood**. All nodes within a *fixed radius* of `x_new` are established as that node's neighborhood. The total cost `total_cost` of reaching `x_new` by going through each neighbor is individually calculated. The neighbor with the lowest `total_cost` is then reassigned as the new parent node of `x_new`.
```python
total_cost = cost(neighbor) + distance(neighbor, x_new)
```
6. **New Node Added to Tree**. `x_new` is now added as a vertex in the tree.
7. **Tree Rewire**. A local optimization is run by computing what the `total_cost` would be for each node in the neighborhood of `x_new` if rewired through `x_new`. If the new cost (`new_cost`) is lower than the current `cost(neighbor)`, the parent of the neighbor node is reassigned to be `x_new` and its cost is updated.
```python
new_cost = cost(x_new) + distance(x_new, neighbor)
```
8. **Goal Connection Check**. If `x_new` is within the preset goal region, the *candidate solution path* from the root node to `x_new` is recorded.
> NOTE: RRT* continues the iterations after a path to the destination is reached. It keeps optimizing the solution over time.

RRT* can be configured to decide how many extra iterations it will perform after the destination is reached in several ways. For example, by defining a maximum number of iterations after the destination is first reached, by a time limit, or by defining a quality threshold for the path. In this project, the RRT* implementation is set to stop after a fixed number of iterations. This ensures a balance between exploration and optimization time. Even after the goal is reached, the algorithm will continue sampling and rewiring nodes to improve path efficiency before handing off the result to the smoothing phase.
