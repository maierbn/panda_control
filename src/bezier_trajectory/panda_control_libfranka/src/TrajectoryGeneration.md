# Trajectory Generation

The overall trajectory is described by a path $\bm p(t): \mathbb R \mapsto \mathbb R^3$. We decouple the spatial planning from the time coordinate, by splitting it into the path itself, which is parametrized by its arc length with a length coordinate $s$ as

$$ \bm p(s) \in \mathbb R^3, \quad s \in [0,L].$$

In a separate step a motion profile is used to generate a smooth mapping $s(t): \mathbb R \mapsto \mathbb R$, connecting the arc length with a time coordinate.

Finally, by inserting the motion profile into the path, we get

$$ \bm p(t) = \bm p(s(t)) \in \mathbb R^3 $$

## Motion Profile

We use a polynomial reference trajectory $s(t)$, assuming no motion at begin and end of the trajectory (i.e. $\dot s(t=\{0, t_E\}) = 0$, $\ddot s(t=\{0, t_E\}) = 0$, ... ). We require the motion profile to be 3 times (velocity, acceleration, jerk) continuously differentiable on $t\in [0, t_E]$ (in fact, a bijective mapping). This leads to a polynomial of order $N = 2*3+1 = 7$. We get (normalized for $s \in [0,1], t\in [0,t_E]$):

$$s(t) = L \sum_{k=4}^7 a_k \left(t/t_E\right)^k = \bm a^T \underbrace{\bm{\bar t}}_{\frac{1}{t_E^k}\begin{bmatrix}t^4\\t^5\\t^6\\t^7 \end{bmatrix}}, \quad\text{with } \bm a = \begin{bmatrix} 35 &  -84 & 70 & -20 \end{bmatrix}^T$$
$$\dot s(t) =  L\sum_{k=4}^7 k a_k t^{k-1}/t_E^k, \quad \ddot s(t) = L\sum_{k=4}^7 k (k-1) a_k t^{k-2}/t_E^k, \quad \ldots$$

To assure, that dynamical limits are kept in Cartesian space, i.e. $\| \bm {\dot p}(t)\| \leq \bm v_{\max}$ and $\| \bm{\ddot p}(t) \|\leq \bm a_{\max}$, the motion profile needs to be transformed. Note, that $\bm p(s)$ is parametrized by arc length, which yields $\|\bm p'(s)\| \equiv 1$. With the cuve derivatives $\bm p'(s), \bm p''(s)$, etc. known, we can express

$$\underbrace{\|\bm p'(s(t))\|}_{=1} \|\dot s(t)\| \leq \bm v_{\max}, \quad t \in [0, t_E] $$
$$\Leftrightarrow \|\dot s(t)\| \leq v_{\max}$$

Solving $\ddot s = 0$ yields the velocity maximum at $t_{\max,v} = t_E/2$, where $\dot s(t_{\max,v}) = \frac{35 L}{16 t_E}$. We get the constraint, that 

$$ t_E \geq \frac{35 L}{16 v_{\max}}.$$

Similarly, for $\ddot s(t)$ we get maxima at $t_{\max,a} = (5 \pm t_E \sqrt 5)/10$, which both yield $\ddot s(t_{\max,a}) \approx_5\mp 0.72143 L/t_E^2$. Hence, the second constraint is

$$ t_E \geq \sqrt{ 7.51319 L /a_{\max}}. $$

By combining both constraints, we can give the end time $t_E$ as

$$ t_E = \max\{35 L/(16 v_{\max}), \sqrt{7.51319 L /a_{\max}} \} .$$