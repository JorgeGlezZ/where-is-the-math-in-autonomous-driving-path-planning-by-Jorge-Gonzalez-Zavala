# Where is the Math in Path Planning for Autonomous Vehicles (Spline Smoothing) by Jorge González Zavala

## Introduction
Path planning is a crucial component in autonomous vehicle navigation. It helps tell the vehicle where and how to move. Not just to reach a destination, but to do so safely, efficiently, and smoothly. Every time I’m stuck in traffic, I can’t help but imagine how much better things would be if cars were driven in a more coordinated and intelligent way. Not by humans who can get tired or distracted, but by machines that can react quickly and precisely, keeping mobility organized and predictable.

<p align = "center">
  <img src = "profile pic.jpg" width = "100" alt = "My Profile Picture">
</p>

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

### Random Trees for Path Planning
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
7. **Tree Rewire**. A local optimization is run by computing the `total_cost` for each node in the neighborhood of `x_new` if rewired through `x_new`. If the new cost (`new_cost`) is lower than the current `cost(neighbor)`, the parent of the neighbor node is reassigned to be `x_new` and its cost is updated.
```python
new_cost = cost(x_new) + distance(x_new, neighbor)
```
8. **Goal Connection Check**. If `x_new` is within the preset goal region, the *candidate solution path* from the root node to `x_new` is recorded.
> NOTE: RRT* continues the iterations after a path to the destination is reached. It keeps optimizing the solution over time.

RRT* can be configured to decide how many extra iterations it will perform after reaching the destination in several ways. For example, by defining a maximum number of iterations after the destination is first reached, by a time limit, or by defining a quality threshold for the path. In this project, the RRT* implementation is set to stop after a fixed number of iterations. This ensures a balance between exploration and optimization time. Even after the goal is reached, the algorithm will continue sampling and rewiring nodes to improve path efficiency before handing off the result to the smoothing phase.

### Discretization
Even if optimized to be short, paths resulting from the RRT* algorithm are not usually realistically drivable. Some extra steps need to be performed on them to make them safe, efficient, and comfortable for an autonomous vehicle.

In this project, it is done by smoothing the path via an optimization-based cost function. The optimization tries to balance **Fidelity** to the original path with geometric smoothness and feasibility.

The focus of this project is on the numerical implementation of the optimization. So, the solutions are to be found using a computer. However, a smooth path is understood to be a continuous function. In other words, a function that is composed of an infinite number of points. Computers, at least the current common ones, work with finite sets of points. Fortunately, the initial path planner already outputs a sparse sequence of points. That should make our job a bit easier. Still, the path needs to become smoother.

Before trying to make the path smoother, the number of points from the rough path is increased by linearly interpolating more points in between the existing ones. This way, our solution is still composed of a finite number of points, but the path will look smoother and more continuous. This naturally discretized domain, indeed, simplifies the problem and sets the stage for numerical optimization.

### From Random Trees to Drivable Paths
Now that our path is formed by enough points, the next step is to smooth it out. Make it more realistic for a vehicle to follow. So, let's dive a little into some of the math behind the smoothing process.

Being the rough path `g` and the optimized path `f`, the cost function is the following:

$$C(f) = w_1 \cdot \text{length}(f) + w_2 \cdot \text{curvature}(f) + w_3 \cdot \text{jerk}(f) + w_4 \cdot \text{fidelity}(f, g)$$

It is noticeable that only one of the terms of the equation depends on `g`, and the rest depend only on `f`. The reason for this is that the only term that will check for **Fidelity** is the fourth term. It will check how different the smooth path is from the rough path. The rest of the terms will check for **Length**, **Curvature**, and **Jerk**. More specifically, they will ensure that the length of the smooth path isn't much different from the length of the rough path, discouraging detours; and that sharp turns and sudden changes in curvature are avoided, ensuring feasible turning and improving both ride stability and comfort. They do so by taking the derivative of the smooth path.

In other words, the cost function will be composed of the zero, first, second, and third derivatives of the function `f`, each penalized via their squared magnitude:

$$J(f) = w_1 \int_L \| f(s) - g(s) \|^2 \, ds + w_2 \int_L \left\| \frac{df}{ds} \right\| \, ds + w_3 \int_L \left\| \frac{d^2f}{ds^2} \right\|^2 \, ds + w_4 \int_L \left\| \frac{d^3f}{ds^3} \right\|^2 \, ds$$

Where all terms depend on `s`, the arc length of the path.

Being our path non-continuous, the derivatives and integrals are **approximated** using finite differences. For example, $f' \approx \frac{f[i+1] - f[i]}{ds}.$

### Weights Determination
$w_1$, $w_2$, $w_3$, and $w_4$ are weights. These weights allow us to adjust how much each term will affect the cost. The higher the weight, the higher the term will increase the cost. Since the optimization aims to minimize the cost function, the higher the weight, the less the term will be allowed to increase, and therefore contribute to the cost.

Each weight's importance may vary depending on the driving context, vehicle dynamics, or passenger sensitivity. Nevertheless, one can estimate the weights for general purposes by aiming for a balance between drivability, obstacle avoidance (amount of deviation from the rough path), and comfort.

In this project, the weights were chosen manually from what was observed during testing as follows: $w_1 = w_2 = w_3~ = 1$; $w_4 = 2$. Keeping length, curvature, and jerk with "equal" importance, while giving more emphasis to the fidelity and ensuring proper obstacle avoidance.

It is worth noting, though, that the weights can also be determined using numerical methods. For this example, a second script was developed using **Bayesian Optimization** (via `scikit-optimize`). A scoring function based on collision risk and smoothness was used to find good weight combinations by evaluating candidate paths.

A few tests clearly showed that the optimal weights are not unique. And that a few changes in the path or the road constraints can allow for very different combinations of weight values to be a relatively good choice. Still, more testing is necessary to reach a definitive conclusion.

Speaking of **unique solutions**, is the path optimization solution unique?

Well, the short answer is no. The function is naturally **non-convex** because it comes from a full dynamic and obstacle-aware case. In fact, the function might have several local minima, and while with our method, convergence is **probable**, it is not guaranteed to a global minimum. To mitigate local minima, our method uses the very same thing that makes the function inherently non-convex, the definition of an initial path, which works as a very well-initialized guess. Other factors that help with the mitigation are the densification of the initial path and running the optimization at multiple intervals.

## Conclusions
### Summary
This project explored the process of turning rough, obstacle-avoiding paths generated by a custom RRT* algorithm into smooth, drivable trajectories using spline-based optimization. The cost function was designed to balance fidelity to the original path with geometric smoothness. Specifically, minimizing path length, curvature, and jerk. The final cost function integrated these elements using finite difference approximations and allowed tuning via weight parameters. The project was focused on the numerical implementation of the path optimization using Python and its libraries, such as *numpy, matplotlib, scipy, and scikit-optimize*.

### My Experience
Before this project, I had a basic notion about path planning and optimization concepts, but not at such a wide scope, and I had never implemented a full pipeline like this. From my math courses, including Calculus I-III, Differential Equations, Linear Algebra, and Numerical Methods, I had a good understanding of derivatives, integration, and cost functions, but I had not worked specifically with spline smoothing in a numerical context.

My approach was to try to understand the math behind the main equation to be used for the numerical implementation before jumping into the code. I focused on first writing clear explanations of what needed to happen in the code, and then gradually implemented each component using Python. Checking every result was crucial, and fixing issues before moving on to the next step. This helped me avoid blind trial-and-error and made the debugging process manageable.

One of the biggest challenges that I faced was balancing time for the theory and implementation equally. Tuning the RRT* algorithm for a more realistic result and then implementing a dynamic solution was probably the hardest and lengthiest part of the practical part. Finding out how to properly tune the weights was a big challenge, too. I used manual tuning at first, then explored Bayesian Optimization to find better values, which revealed how non-unique but context-sensitive the weight selection is.

I lost a decent-sized chunk of my work due to a computer crash, which was frustrating, but having to redo that part of the work was also useful to clarify some concepts better. That part was unexpectedly helpful.

### Future Perspective
Going forward, I would like to improve the dynamic part of the algorithm and extend the optimization to higher dimensions, including constraints like velocity and acceleration. I am also interested in making the weights adaptable in real-time, depending on the environment or driving goals. For example, a path in an urban environment might prioritize fidelity and jerk, while an off-road vehicle might favor shorter or more aggressive curves. Some code cleaning and optimizing would be very useful for computational speed and power, too.

This project gave me a solid foundation in both the mathematical and practical sides of path smoothing. I feel confident about expanding it into more realistic driving systems in the future.
