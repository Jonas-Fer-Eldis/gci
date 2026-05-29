# 2D Aircraft Intercept Algorithm

## Known Inputs

| Variable | Description |
|----------|-------------|
| $P_i$ | Interceptor current position $(x, y)$ |
| $P_t$ | Target current position $(x, y)$ |
| $V_i$ | Interceptor speed (scalar) |
| $\vec{V}_t$ | Target velocity vector $(v_x, v_y)$ |
| $H_i$ | Interceptor current heading |
| $R$ | Turn radius of interceptor |

---

## Step 1 — Determine Exit Point on Turning Circle

For each candidate arc angle $\alpha$ (sampled over feasible range, e.g., $0°$ to $180°$ for both left and right turns):

$$T_{turn} = \frac{R \cdot |\alpha|}{V_i}$$

- $P_{exit}$ = interceptor position on the turning circle at angle $\alpha$
- $H_{exit}$ = heading at exit (tangent to circle at $P_{exit}$)

---

## Step 2 — Advance Target by Turn Time

$$P_t' = P_t + \vec{V}_t \cdot T_{turn}$$

This is the target's position at the moment the interceptor exits the turn.

---

## Step 3 — Solve for Intercept Time on Straight Leg

After exiting the turn, the interceptor flies straight at speed $V_i$ toward an intercept point. The target continues from $P_t'$ with constant velocity $\vec{V}_t$.

The intercept condition (both arrive at the same point at the same time):

$$|P_t' - P_{exit} + \vec{V}_t \cdot t|^2 = (V_i \cdot t)^2$$

Let $\vec{D} = P_t' - P_{exit}$ (relative position vector). Expand:

$$|\vec{D}|^2 + 2 \cdot (\vec{D} \cdot \vec{V}_t) \cdot t + |\vec{V}_t|^2 \cdot t^2 = V_i^2 \cdot t^2$$

Rearrange into quadratic $at^2 + bt + c = 0$:

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

---

## Step 5 — Select Optimal Exit Angle

Choose $\alpha$ that minimizes total intercept time:

$$T_{total} = T_{turn} + t_{straight}$$

### Practical approach:
- Sample $\alpha$ over the feasible arc
- Compute $T_{total}$ for each sample
- Pick the $\alpha$ with minimum $T_{total}$

Alternatively, use golden section search or Newton's method — $T_{total}(\alpha)$ is smooth and typically unimodal.

---

## Guarantees

| Condition | Guarantee |
|-----------|-----------|
| $V_i > |\vec{V}_t|$ | Positive root exists for **all** exit points on the circle |
| $V_i = |\vec{V}_t|$ | Positive root exists only if target has a closing component |
| $V_i < |\vec{V}_t|$ | Solution may not exist |

When $V_i > |\vec{V}_t|$, the discriminant $b^2 - 4ac = 4(\vec{D} \cdot \vec{V}_t)^2 + 4(V_i^2 - |\vec{V}_t|^2)|\vec{D}|^2$ is always positive, and one root is always positive. The algorithm is guaranteed to find an intercept from any exit point.

---

## Why This Is Exact

Step 3 solves the geometric intercept problem directly — no iteration, no closing speed approximation. The quadratic gives the exact time for a straight-line intercept from a given point against a constant-velocity target. The only approximation is the discretization of exit angle $\alpha$ in Step 5, which can be made arbitrarily precise.
