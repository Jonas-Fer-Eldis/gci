# 2D Aircraft Intercept Algorithm
 
## Known Inputs

| Variable | Description |
|----------|-------------|
| P_i | Interceptor current position (x_i, y_i) |
| P_t | Target current position (x_t, y_t) |
| V_i | Interceptor speed (scalar) |
| V_t | Target velocity vector (dx_t, dy_t) |
| H_i | Interceptor current heading (radians) |
| R | Turn radius of interceptor |
| C | Center of turning circle (c_x, c_y) |

---

## Parametric Equations of Motion

### Interceptor — Turning Circle

The interceptor follows a circular arc centered at C with radius R:

```
x(t) = c_x + R * cos(θ₀ + ω * t)
y(t) = c_y + R * sin(θ₀ + ω * t)
```

where:

- θ₀ = atan2(y_i - c_y, x_i - c_x)
- ω = ±V_i / R (positive for left turn, negative for right turn)
- t ranges from 0 to 2πR / V_i (full circle)

The heading at any point on the circle is tangent to the arc:

```
H(t) = θ₀ + ω * t + (π/2) * sign(ω)
```

### Target — Straight Line

The target follows a constant-velocity straight line:

```
x(t) = x_t + dx_t * t
y(t) = y_t + dy_t * t
```

where t ranges from 0 to infinity.

---

## Step 1 — Parametrize Exit Point

Choose an arc parameter t_arc in the range [0, 2πR / V_i]. This defines:

```
P_exit = ( c_x + R * cos(θ₀ + ω * t_arc),
           c_y + R * sin(θ₀ + ω * t_arc) )

T_turn = t_arc
```

---

## Step 2 — Advance Target by Turn Time

```
P_t' = ( x_t + dx_t * T_turn,
         y_t + dy_t * T_turn )
```

---

## Step 3 — Solve for Intercept Time on Straight Leg

Let D = P_t' - P_exit (relative position at moment of exit).

The intercept condition (both arrive at same point at same time):

```
|D + V_t * t|² = (V_i * t)²
```

Expand into quadratic at² + bt + c = 0:

```
a = V_i² - |V_t|²
b = -2 * dot(D, V_t)
c = -|D|²
```

Solve:

```
discriminant = b² - 4*a*c
t_straight = (-b - sqrt(discriminant)) / (2*a)
```

Take the smallest positive root.

---

## Step 4 — Compute Intercept Point and Heading

```
P_intercept = P_t' + V_t * t_straight

heading = (P_intercept - P_exit) / |P_intercept - P_exit|

T_total = T_turn + t_straight
```

---

## Step 5 — Find Optimal Exit Point

**Objective:** Minimize T_total(t_arc) = t_arc + t_straight(t_arc)

### Algorithm: Golden Section Search

Finds the minimum of a unimodal function on a bounded interval.

**Inputs:**

- Interval [a, b] = [0, 2πR / V_i]
- Tolerance ε
- Golden ratio φ = (1 + sqrt(5)) / 2

**Procedure:**

```
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

**Note:** If T_total is not unimodal (rare, can occur in close-range scenarios), use uniform sampling (e.g., 100 points) followed by local refinement around the best sample.

---

## Guarantees

### Case 1: V_i > |V_t| (interceptor faster than target)

- a = V_i² - |V_t|² > 0
- c = -|D|² is always negative or zero
- discriminant = b² - 4ac is always positive
- One root is always positive

**Result:** A positive t_straight exists for every exit point on the circle. The algorithm is guaranteed to find an intercept.

### Case 2: V_i = |V_t| (equal speeds)

Intercept exists only if the target has a closing component toward the interceptor. Some exit points may yield no valid solution.

### Case 3: V_i < |V_t| (target faster)

Intercept may be geometrically impossible. No guarantee of a solution on any exit point.

---

## Why This Is Exact

Step 3 solves the geometric intercept problem directly — no iteration, no closing speed approximation. The quadratic gives the exact time for a straight-line intercept from a given point against a constant-velocity target. The only approximation is the resolution of the exit angle search in Step 5, which can be made arbitrarily precise.
