# Native-backed static-contact comparison

Status: **bounded model result; no integrated robot or hardware retention acceptance**.

The separately saved [native track/tread candidate](../integrated/TRACK_TREAD_CANDIDATE.md) clears its sampled floor corridor, but the existing regularized pressure/friction law drops a staged NOTE. This additive experiment uses MuJoCo's built-in elliptic contact solver and NoSlip post-processing to test a formulation that can sustain static friction. Historical pressure code, traces, native archives and the driver are unchanged.

## What was actually simulated

The final relieved F3D hash is required at launch. The fixture uses its two inner straight belt faces at absolute Y176.422393 mm, their native 12.7 mm height and the 3.5 mm combined belt/tread material thickness. The face dimensions come from the previously verified native candidate and its original source generator; an additional direct Fusion face-surface check has not completed and is not claimed. The fixture is translated upward to isolate gravity release from the floor. It omits the curved return loops and every other robot part.

The NOTE is a free six-DOF body with the inherited 0.235 kg uniform-torus mass/inertia hypothesis. A ring of 128 or 256 capsules approximates its torus; maximum centerline sagitta is 0.045900 or 0.011475 mm. Explicit inertia prevents geometry mass double-counting. There is no floor, actuator, equality constraint, NOTE attachment or state overwrite after initialization. MuJoCo alone advances state.

The normal stiffness is fitted with gravity and friction disabled to the prior uncalibrated pressure model's initial 3.764848 N per side. It is never fitted to a hold/drop outcome. This is a comparison against a hypothesis, not hardware calibration. Point-contact normal response differs from finite-area pressure away from that initial pose; geometric refinement does not validate the pressure distribution.

MuJoCo documents NoSlip as post-processing that suppresses friction drift from soft constraints. It also has tradeoffs, including ill-defined inverse dynamics and possible chatter in overconstrained grasps. This experiment uses 20 iterations and checks observed forces, friction cones and warnings; it does not establish a general choice for the complete robot. See the [official NoSlip option](https://mujoco.readthedocs.io/en/latest/XMLreference.html#option-noslip-iterations) and [contact modeling guidance](https://mujoco.readthedocs.io/en/latest/modeling.html).

## Results

Eleven source-bound runs pass the saved-trace verifier. All use MuJoCo 4.0.1 and the same native archive. No engine warnings or measured friction-cone violations occur.

| Experiment | Observed result |
| --- | --- |
| Five-second dwell, 128 capsules, 0.2 ms | Maximum vertical drift 1.44e-9 m; remains supported |
| Five-second dwell, 256 capsules, 0.2 ms | Maximum vertical drift 4.03e-8 m; remains supported |
| Half timestep, 128 capsules, one second | Remains supported; force balances NOTE weight |
| Initial downward velocity -0.1 m/s | Stops after 3.554 mm downward movement; final vertical speed below 1e-7 m/s |
| Friction coefficient 0.1 | Drops 50 mm in 0.1856 s |
| Zero-friction and no-contact controls | Drop 50 mm in approximately 0.101 s |
| NoSlip disabled, identical other parameters | Drifts 1.878 mm in one second and 10.978 mm in five seconds |

Steady contact force balances the 2.304563 N weight. The no-contact trace matches analytic semi-implicit Euler free fall below 1e-12 m. The five-second runs took approximately 21 seconds at 128 capsules and 43 seconds at 256 under the observed execution conditions; these are measured offline results, not real-time guarantees.

The no-slip on/off comparison isolates a solver change within the same point-contact model. The comparison with the earlier finite-area model changes both contact formulation and discretization, so it cannot attribute all differing behavior to one parameter. The earlier failed traces remain valid evidence about their original law. Friction-cone and steady force-balance checks are not a complete compliance-energy validation.

## Decision and integration boundary

Built-in MuJoCo contact is a viable formulation to investigate for retention, with demonstrated static support and finite-friction failure controls. There is no basis to reject the physical candidate solely because the previous regularized law creeps. There is also no basis to claim real retention from this model.

The next robot integration must model the complete native belt path, finite motor/rotor reaction and moving cradle, then acquisition and launcher transfer. Fixed belt surfaces in this fixture are an external support boundary; connecting them rigidly to the driver would silently assume locked belts. That would not prove the actual mechanism's transport behavior. Native pose/contact registration outside this straight fixture, measured friction/compliance, structural qualification and shared actuation remain missing. No drive or shooting demo is claimed.

## Run

Use the existing MuJoCo/NumPy environment and fresh private output directories; no installation is performed.

```powershell
<mujoco-python> -B sim/mujoco/native_static_contact.py --project . --native-archive <private-final-relieved.f3d> --output <fresh-private-output> --duration 5
```

Use `--segments 256`, `--dt .0001`, `--mu .1`, `--no-contact`, `--initial-vz -.1`, or `--noslip 0` for the documented comparisons. The saved [sanitized evidence](evidence/native-static-contact-20261118.json) includes all parameters and trace hashes. Full traces and generated XML remain in the private packet. No GUI operation or merge was performed.
