# 2D Aircraft Intercept Algorithm

## Known Inputs

| Variable | Description |
|----------|-------------|
| $P_i = (x_i, y_i)$ | Interceptor current position |
| $P_t = (x_t, y_t)$ | Target current position |
| $V_i$ | Interceptor speed (scalar) |
| $\vec{V}_t = (dx_t, dy_t)$ | Target velocity vector |
| $H_i$ | Interceptor current heading (radians) |
| $R$ | Turn radius of interceptor |
| $C = (c_x, c_y)$ | Center of turning circle |

---

## Parametric Equations of Motion

### Interceptor — Turning Circle

The interceptor follows a circular arc centered at $C$ with radius $R$:

$$x(t) = c_x + R \cos(\theta_0 + \omega t)$$

$$y(t) = c_y + R \sin(\theta_0 + \omega t)$$

where:
- $\theta_0$ is the initial angle of the interceptor on the circle: $\theta_0 = \text{atan2}(y_i - c_y,\ x_i - c_x)$
- $\omega = \pm V_i / R$ (positive for left turn, negative for right turn)
- $t \in [0,\ 2\pi R / V_i]$ covers a full circle

The heading at any point on the circle is tangent to the arc:

$$H(t) = \theta_0 + \omega t + \frac{\pi}{2} \cdot \text{sign}(\omega)$$

### Target — Straight Line

The target follows a constant-velocity straight line:

$$x(t) = x_t + dx_t \cdot t$$

$$y(t) = y_t + dy_t \cdot t$$

where $t \in [0, \infty)$.

---

## Step 1 — Parametrize Exit Point

Choose an arc parameter $t_{arc} \in [0,\ 2\pi R / V_i]$. This defines:

$$P_{exit} = \bigl(c_x + R\cos(\theta_0 + \omega \cdot t_{arc}),\quad c_y + R\sin(\theta_0 + \omega \cdot t_{arc})\bigr)$$

$$T_{turn} = t_{arc}$$

---

## Step 2 — Advance Target by Turn Time

$$P_t' = (x_t + dx_t \cdot T_{turn},\quad y_t + dy_t \cdot T_{turn})$$

---

## Step 3 — Solve for Intercept Time on Straight Leg

Let $\vec{D} = P_t' - P_{exit}$ (relative position at moment of exit).

The intercept condition:

$$|\vec{D} + \vec{V}_t \cdot t|^2 = (V_i \cdot t)^2$$

Expand into quadratic $at^2 + bt + c = 0$:

$$a = V_i^2 - |\vec{V}_t|^2$$

$$b = -2 \cdot (\vec{D} \cdot \vec{V}_t)$$

$$c = -|\vec{D}|^2$$

Solve:

$$t_{straight} = \frac{-b - \sqrt{b^2 - 4ac}}{2a}$$

Take the smallest positive root.

---

## Step 4 — Compute Intercept Point and Heading

$$P_{intercept} = P_t' + \vec{V}_t \cdot t_{straight}$$

$$\hat{h}_{straight} = \frac{P_{intercept} - P_{exit}}{|P_{intercept} - P_{exit}|}$$

$$T_{total} = T_{turn} + t_{straight}$$

---

## Step 5 — Find Optimal Exit Point

**Objective:** Minimize $T_{total}(t_{arc}) = t_{arc} + t_{straight}(t_{arc})$

### Algorithm: Golden Section Search

This finds the minimum of a unimodal function on a bounded interval.

**Inputs:**
- Interval $[a, b] = [0,\ 2\pi R / V_i]$ (consider both left and right turn circles)
- Tolerance $\epsilon$
- $\varphi = \frac{1 + \sqrt{5}}{2}$ (golden ratio)

**Procedure:**

```
φ = (1 + √5) / 2
resphi = 2 - φ

a = 0
b = 2πR / V_i

x1 = a + resphi * (b - a)
x2 = b - resphi * (b - a)
f1 = T_total(x1)
f2 = T_total(x2)

while |b - a| > ε:
    if f1 < f2:
        b = x2
        x2 = x1
        f2 = f1
        x1 = a + resphi * (b - a)
        f1 = T_total(x1)
    else:
        a = x1
        x1 = x2
        f1 = f2
        x2 = b - resphi * (b - a)
        f2 = T_total(x2)

t_arc_optimal = (a + b) / 2
```

**Run this for both left and right turn circles**, then pick the overall minimum.

**Note:** If $T_{total}$ is not unimodal (rare, can occur in close-range scenarios), use uniform sampling (e.g., 100 points) followed by local refinement around the best sample.

---

## Guarantees

When $V_i > |\vec{V}_t|$ (interceptor faster than target):

- $a = V_i^2 - |\vec{V}_t|^2 > 0$
- $c = -|\vec{D}|^2 \leq 0$
- Discriminant: $b^2 - 4ac = 4(\vec{D} \cdot \vec{V}_t)^2 + 4(V_i^2 - |\vec{V}_t|^2)|\vec{D}|^2 > 0$
- One root is always positive

**Result:** A positive $t_{straight}$ exists for every exit point on the circle. The algorithm is guaranteed to find an intercept.

When $V_i = |\vec{V}_t|$ (equal speeds):

- Intercept exists only if the target has a closing component toward the interceptor
- Some exit points may yield no valid solution

When $V_i < |\vec{V}_t|$ (target faster):

- Intercept may be geometrically impossible
- No guarantee of a solution on any exit point

---

## Why This Is Exact

Step 3 solves the geometric intercept problem directly — no iteration, no closing speed approximation. The quadratic gives the exact time for a straight-line intercept from a given point against a constant-velocity target. The only approximation is the resolution of the exit angle search in Step 5, which can be made arbitrarily precise.
