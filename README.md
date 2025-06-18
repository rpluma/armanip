# Ampliación de Robótica - Manipuladores

## Cuadernos de Jupyter

- [Resumen de ecuaciones](jupyter/Ecuaciones.ipynb)
- [Comandos frecuentes](jupyter/FAQ.ipynb)
- [Lab 1: Cartesian trajectory planning](jupyter/lab1/lab1.ipynb)
- [Lab 2: Manipulator dynamics simulation](jupyter/lab2/lab2.ipynb)
- [Lab 3: Inverse dynamics control](jupyter/lab3/lab3.ipynb)
- [Lab 4 Impedance Control](jupyter/lab4/lab4.ipynb)
- [Lab 5 Force Control](jupyter/lab5/lab5.ipynb)

## Estructura del repositorio

- advanced_robotics_ws: subcarpeta con prácticas de ros2
- jupyter: subcarpetas con cuadernos jupyter
- matlab: subcarpeta con módulos matlab y simulink

## Resumen de ecuaciones

### Lab 1: Cartesian trajectory planning

Time law from initial time of the trajectory $t_i$ and final time of the trajectory $t_f$

$$\lambda(t) = \frac {t - t_i} {t_f - t_i} \quad \lambda \in [0, 1] \tag {1.1}$$

Linearly interpolated transition:

$$p(t) = p_0 + \lambda(t) (p_1 - p_0) \tag {1.2}$$

Quaternion as a rotation angle $\theta$ over an axis $\vec n$:

$$q = [\omega, \vec v] = Rot(n, \theta) = [\cos \frac \theta 2 + \sin \frac \theta 2 \vec n ] \quad  \omega \in \mathbb{R} \quad \vec v, \vec n \in \mathbb{R}^3  \tag {1.3a}$$

$$ \quad \theta = 2 \arccos(\omega) \quad \vec n = \frac {\vec v} {\sin(\theta /2)} \tag {1.3b}$$

3D rotation to go from $q_A$ to $q_B$:

$$q_A \cdot q_C = q_B \implies q_C = q_A^{-1} \cdot q_B = [w_c, \vec v_C] \tag {1.4}$$

Interpolation of rotation:

$$\theta_{\lambda} (t) = \lambda(t) \theta \tag {1.5}$$

Interpolation of the rotation quaternion:

$$q_{rot}(\lambda) = [w_{rot} (\lambda), \vec v_{rot} (\lambda)] \quad 
w_{rot} (\lambda) = \cos \frac {\theta_{\lambda}(\lambda)} {2} \quad \vec v_{rot}(\lambda) = \vec n \sin \frac {\theta_{\lambda}(\lambda)} {2} \tag {1.6}$$

Interpolation quaternion $q_{\lambda}(\lambda)$:

$$q_{\lambda}(\lambda) = q_A \cdot q_{rot}(\lambda) \implies q_{\lambda} (0) = q_A \quad q_{\lambda} (1) = q_B \tag {1.7}$$

### Lab 2: Manipulator dynamics simulation

Dynamics of an open kinematic chain robotic manipulator:

$$M(q)\ddot q + C(q, \dot q)\dot q + F_b \dot q + g(q) = \tau + \tau_{ext} \tag {2.1}$$

$$\ddot q = M^{-1}(q)[\tau + \tau_{ext} - C(q, \dot q)\dot q - F_b \dot q - g(q)] \tag {2.2}$$

Where:

- $q, \dot q, \ddot q$: vectors of joint positions, velocities and accelerations
- $M(q)$: intertia matrix
- $C(q, \dot q)$: Coriolis and centrifugal forces matrix
- $F_b$: viscous friction matrix
- $g$: gravity vector
- $\tau$: commaned join torques
- $\tau_{ext}$: torques due to external forces

$$M(q) = \begin{bmatrix}
m_1  l_1^2 + m_2 (l_1^2+2  l_1 l_2  \cos(q2)+ l_2^2) & m_2 (l_1 l_2 cos(q_2) + l_2^2)\\
m_2  (l_1  l_2  \cos (q_2) + l_2^2) & m_2  l_2^2
\end{bmatrix} \tag {2.3}$$

$$C(q,\dot q)\dot q = \begin{bmatrix}
-m_2 l_1 l_2 \sin(q_2) (2 \dot q_1 \dot q_2 + \dot q_2^2)\\
m_2 l_1 l_2 \dot q_1^2 \sin(q_2)
\end{bmatrix} \tag {2.4}$$

$$F_b = \begin{bmatrix}
b_1 & 0\\
0 & b_2
\end{bmatrix} \tag {2.5}$$

$$g(q) = \begin{bmatrix}
(m_1 + m_2) l_1 g \cos(q1)+m_2 l_2 g \cos (q_1 + q_2)\\
m_2 l_2 g \cos(1_1 + q_2)
\end{bmatrix} \tag {2.6}$$

$$J(q) = \begin{bmatrix}
-l_1 \sin(q_1) -l_2 \sin(q_1 + q_2) & -l_2 \sin(q_1 + q_2)\\
 l_1 \cos(q_1) -l_2 \sin(q_1 + q_2) &  l_2 \cos(q_1 + q_2)
\end{bmatrix} \tag {2.7}$$

$$\tau_{ext} = J(q)^T \cdot F_{ext} \tag {2.8}$$

### Lab 3: Inverse Dynamics Control

Gravity compensation:

$$\tau = g(q) \tag {3.1}$$

Simulation of the force sensor:

$$\tau_{ext} = J(q)^T \cdot F_{ext} \tag {2.8}$$


Linearization by inverse dynamics control:

$$\tau = M(q)\cdot \ddot q_d + C(q, \dot q)\dot q + F_b \dot q + g(q) \tag {3.2}$$

Linearization by inverse dynamics control including external forces

$$\tau = M(q)\cdot \ddot q_d + C(q, \dot q)\dot q + F_b \dot q + g(q) + J(q)^T \cdot F_{ext} \tag {5.1}$$