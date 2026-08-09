# GNSS and IMU Data Synthesis Algorithm

The algorithm produces synchronized GNSS and IMU data from vehicle telemetry.
It reconstructs the vehicle trajectory from the velocity vector components, constructs
a Direction Cosine Matrix (DCM) from the attitude angles, and then calculates gyroscope
and accelerometer readings in the body frame. The gyroscope readings in the body frame
are refined iteratively to reduce data synthesis error caused by the simplified DCM
kinematics equation used by the navigation algorithm.

> **WARNING**
> 
> Before using the algorithm, clean the source data of non-numeric values and resample it
> onto a common uniform time grid.

---
## Notation and Conventions

| Notation           | Description                                                                                  |
|--------------------|----------------------------------------------------------------------------------------------|
| `ENU`              | *East, North, Up*. Navigation coordinate system (local-level frame).                         |
| `nav`, `n`         | ENU navigation frame. `nav` is used in variable names; `n` is used in matrix notation.       |
| `body`, `b`        | Coordinate system rigidly attached to the vehicle.                                           |
| `DCM`              | *Direction Cosine Matrix*.                                                                   |
| `C_b^n`, `C_b^nav` | Transformation matrix for vector components from the body frame to the ENU navigation frame. |
| `ψ`                | Heading / yaw.                                                                               |
| `ϑ`                | Pitch.                                                                                       |
| `γ`                | Roll.                                                                                        |
| `φ`                | Geographic latitude.                                                                         |
| `λ`                | Geographic longitude.                                                                        |
| `h`                | Altitude.                                                                                    |
| `V_nav`            | Velocity vector in the ENU navigation frame.                                                 |
| `ω_b`              | Angular rate of the body frame.                                                              |
| `ω_nav`            | Angular rate of the navigation frame.                                                        |
| `U_nav`            | Earth rotation rate projected onto the navigation frame.                                     |
| `g`                | Gravity vector in the navigation frame.                                                      |
| `[V]`              | Skew-symmetric matrix corresponding to vector `V`.                                           |
| `GS(С)`            | Orthonormalization of matrix `С` using the Gram–Schmidt method.                              |
| `ansim(C)`         | Extraction of the antisymmetric part of matrix `C`.                                          |
| `vee(C)`           | Conversion of skew-symmetric matrix `C` to a three-component vector.                         |
| `ε`                | Acceptable DCM consistency error.                                                            |

> **NOTE**
> 
> See [Coordinate Systems, Attitude and Navigation Parameters](conventions.md) for
> details of the coordinate systems, attitude parameters, navigation parameters, and
> units of measurement used by the project.

---
## Data Generation Algorithm

![alg-nav-inv.png](../.draw/alg-nav-inv.png)

---
## Input Data

The source telemetry is provided as discrete samples:
$$
t_k, \qquad k = 0, 1, 2, \ldots, N.
$$

The minimum required data is identified in items 1–3. The data in item 4 is intended
for future simulation of an air data system (СВС) and a Doppler velocity sensor; it is
not used by this library.

1) Velocities in the navigation coordinate system
$$
V_x^{nav}(t_k), \qquad
V_y^{nav}(t_k), \qquad
V_z^{nav}(t_k).
$$

2) Euler–Krylov attitude angles
$$
\psi(t_k), \qquad
\gamma(t_k), \qquad
\vartheta(t_k).
$$

3) Initial geographic coordinates
$$
\varphi(t_0), \qquad
\lambda(t_0), \qquad
h(t_0).
$$

4) Optional data that may be used to simulate additional aircraft instruments in the future
$$
V_{IAS}(t_k), \qquad
\alpha(t_k), \qquad
\beta(t_k), \qquad
h^{baro}(t_k), \qquad
V_{GS}(t_k), \qquad
Yгол Cноса(t_k).
$$

---
## Input Data Preprocessing

### Interpolation of Invalid Values

Replace `NaN`, `null`, and `inf` values with interpolated values from the
corresponding time series before passing the data to the algorithm.

### Heading Angle Unwrapping

Unwrap the heading time series:

$$
\psi(t_k) \rightarrow \operatorname{unwrap}(\psi(t_k)).
$$

Remove discontinuities caused by crossing the angular period boundary.

### Determining the Sampling Interval

Determine the common time step as the median of the source time intervals:

$$
\Delta t = \operatorname{median}\left(t_{k+1} - t_k\right),
\qquad k = 0, \ldots, N - 2.
$$

### Resampling

Construct a uniform time grid:

$$
t_{i+1} = t_i + \Delta t,
\qquad i = 0, \ldots, N - 2.
$$

Interpolate every time series used by the algorithm onto this grid. Ensure that
velocity, attitude angle, and geographic parameter values correspond to the same
time instants `t_i`.

---
## Reconstructing the Vehicle Trajectory

The project defines the navigation frame as ENU:
$$
V_{ox} = V_x^{nav}, \qquad
V_{oy} = V_y^{nav}, \qquad
V_{oz} = V_z^{nav}.
$$

Latitude is computed by integrating the north velocity component:
$$
\varphi_i =
\varphi_{i-1} + \Delta t\frac{V_{oy,i-1}}{R_{\varphi} + h_{i-1}}.
$$

Longitude is computed by integrating the east velocity component:
$$
\lambda_{i+1} = \lambda_{i-1} + \Delta t\frac{V_{ox,i-1}}
{\left(R_{\lambda} + h_{i-1}\right)\cos\left(\varphi_{i-1}\right)}.
$$

Altitude is computed by integrating the vertical velocity component:
$$
h_i = h_{i-1} + \Delta t V_{oz,i-1}.
$$

Here, `R_φ` and `R_λ` are the Earth-model radii of curvature used to convert
linear displacement into changes in latitude and longitude.

---
## DCM Construction

For each time instant `t_i`, the attitude matrix is constructed from the heading,
roll, and pitch angles:

$$
C_b^{nav}(t_i)
=
C_b^{nav}\left(\psi(t_i),\gamma(t_i),\vartheta(t_i)\right).
$$

`C_b^nav` transforms vector components from the body frame to the ENU navigation frame.
The inverse transformation uses the transposed matrix:

$$
v_b = \left(C_b^{nav}\right)^T v_{nav}.
$$

---
## Computing the Gravity Vector

The gravity vector is calculated from the current geographic coordinates:

$$
g(t_i) = g\left(\varphi(t_i), h(t_i)\right).
$$

---
## Initial Estimate of Gyroscope Readings

The change in DCM over interval `Δt` is used to calculate an initial estimate of
the IMU gyroscope readings in the body frame:

$$
[\omega_b(t_i)] = \left(C_b^{nav}(t_i)\right)^T \frac{C_b^{nav}(t_{i+1}) - C_b^{nav}(t_i)}{\Delta t} + 
\left(C_b^{nav}(t_i)\right)^T [\omega_{nav}(t_i)] C_b^{nav}(t_i).
$$

This expression is the inverse DCM kinematics equation for two rotating frames.

The following initial conditions are used for iterative refinement of the gyroscope readings:
$$
j = 0,
\qquad
\omega_b^j = \omega_b(t_i).
$$

---
## Iterative Refinement of Gyroscope Readings

The estimate of gyroscope readings `ω_b^j` at iteration `j` is used to predict
the Direction Cosine Matrix at the next step:
$$ 
\left(C_b^{nav}(t_{i+1})\right)_j = C_b^{nav}(t_i) + 
\Delta t \left(C_b^{nav}(t_i)[\omega_b^j] - 
[\omega_{nav}(t_i)]C_b^{nav}(t_i)\right).
$$

The current estimate `ω_b^j` therefore predicts the DCM state at time `t_{i+1}`.

The resulting matrix is orthonormalized using the Gram–Schmidt method:
$$
DCM^{pred}
=
GS\left(\left(C_b^{nav}(t_{i+1})\right)_j\right).
$$

The true DCM at time `t_{i+1}` is calculated from the input attitude angles:

$$
DCM^{true} = C_b^{nav}(t_{i+1}).
$$

The prediction error is defined as the difference between the true and predicted DCMs:

$$
\Delta C^j
=
\left|DCM^{true} - DCM^{pred}\right|.
$$

If the difference between the matrices is sufficiently large, the gyroscope readings
that caused the least-squares calculation error are corrected.

$$
\Delta C^j < \varepsilon,
$$

If the condition above is not met, a correction matrix is calculated:

$$
V
=
\operatorname{ansim}
\left(
\left(C_b^{nav}(t_i)\right)^T
\frac{\Delta C^j}{\Delta t}
\right).
$$

$$
\delta\omega_b^j = \operatorname{vee}(V).
$$

The next approximation of the gyroscope readings is:

$$
\omega_b^{j+1}
=
\omega_b^j + \delta\omega_b^j.
$$

The iteration counter is then incremented:

$$
j = j + 1.
$$

The calculation of `DCM^pred`, `ΔC^j`, and the correction is repeated until
`ΔC^j < ε`.

---
## Calculating Accelerometer Readings

The velocity derivative in the navigation frame is calculated by finite differences:

$$
\dot V_{nav}(t_i) = \frac{V_{nav}(t_{i+1}) - V_{nav}(t_i)}{\Delta t}.
$$

Accelerometer readings in the body frame are calculated as:

$$
a_b(t_i)
=
\left(C_b^{nav}(t_i)\right)^T
\left[
\dot V_{nav}(t_i) +
\left([2\omega_{nav}(t_i)] + [U_{nav}(t_i)]
\right)V_{nav}(t_i) -
g(t_i)
\right].
$$

The transposed DCM transforms the result from the navigation frame to the body frame.
The expression includes the velocity derivative, navigation-frame angular terms,
and the gravity vector.

---
## Output Data

### GNSS

The following GNSS data is produced:

$$
V_x^{nav}(t_i), \qquad
V_y^{nav}(t_i), \qquad
\varphi(t_i), \qquad
\lambda(t_i), \qquad
h(t_i).
$$

This data set is used as a correction source for the IRS navigation system. It can
be used to correct an IRS under development by altitude, velocity, and position.

### IMU

The following data is produced:

$$
a_b(t_i), \qquad
\omega_b(t_i).
$$

GNSS and IMU data are generated on the same time grid with a fixed step `Δt`.

---
## Summary of Steps

1. Read the source telemetry and initial geographic coordinates.
2. Preprocess the input data: interpolate invalid values, unwrap the heading angle,
   and resample onto a uniform time grid.
3. Reconstruct geographic coordinates `φ`, `λ`, and `h` by integrating velocity
   components in the local-level frame.
4. Construct `C_b^n` from angles `ψ`, `ϑ`, and `γ` for each time instant, and calculate `g(φ, h)`.
5. Calculate the initial estimate of vehicle angular rate measured by the gyroscopes, `ω_b`, from the DCM change.
6. Refine gyroscope readings `ω_b` iteratively: predict the DCM at the next step, orthonormalize it,
   determine the error relative to the DCM constructed from telemetry, and correct `ω_b` until the
   required accuracy condition is met.
7. Calculate the velocity derivative `V̇_nav` and accelerometer readings `a_b` in the body frame.
8. Generate GNSS and IMU output data sets on the common time grid.
---