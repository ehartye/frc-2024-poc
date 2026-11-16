# 2024 mission supervisor interface scaffold

Local development only. This additive controller has not been connected to a physical acquisition plant, deployed robot or field sensor adapter. Injected passing observations are test inputs, not measured success.

## Serialized baseline

`WAIT_START -> DRIVE_SHOT -> ALIGN_SHOT -> SPINUP -> RELEASE -> SEEK_NOTE -> ACQUIRE -> INDEX -> STOW -> RETURN_SHOT -> ALIGN_SHOT -> SPINUP -> RELEASE -> DONE`.

The preload must be reported present, indexed and stowed. A release completes only when an absent piece and downstream exit observation are both reported while alignment, stopped state and RPM remain valid. This counts controller release observations, not scores or proof of capture. A future sensor adapter must establish event validity; a stuck-high exit signal is not independently diagnosed by this scaffold.

Drive/align and mechanism motor requests are serialized. Concurrent spinup while driving is intentionally deferred as an alternative, not silently assumed. No existing plant, runtime or controller was edited.

## Input contract

`Observation`: strictly increasing sequence/timestamp, piece enum PRESENT/ABSENT/UNKNOWN, indexed/stowed state, named reached goal, alignment, stopped state, floor target visibility, jam flag, shooter RPM and downstream exit observation. `update(now, observation, start, requestDegradedPlan)` returns explicit state/fault and actuator/goal requests.

Adapters must only assert a reached goal for the currently requested goal, and validate physical meaning of each flag. Fresh timestamps do not prove truthful sensors. Unknown piece or stale/repeated/nonfinite observations inhibit all requests by latching FAULT. There is no autonomous recovery from uncertain possession.

## Output contract

Output requests named SHOT_POSE or FLOOR_NOTE motion, shot alignment, intake deployment/stow, intake/index duty, shooter target RPM and release duty. Adapters must map these to actual controllers and independently enforce limits. Zero software demand is not proof of stopping, retention, safe gravity behavior or power-loss safety. Fields do not specify installed actuators, calibration, field transforms or force models.

## Bounded recovery

One acquisition/index timeout retry per mission; a second timeout faults. One jam reversal per mission, with configurable duration; another jam faults. A reverse that ejects the piece returns to acquisition, not presumed possession. Low-RPM timeout latches LOW_RPM. Only an explicit operator request moves this fault to DEGRADED_PLAN, which still produces no motion, motor or release commands. No shorter-distance shot, speed calibration or automatic reduced-range fallback is invented.

All Config values are hypothetical, including 2500 RPM readiness, demand fractions and timing. The sample hypothesis uses maximum observation age 50 ms, update gap 100 ms, 200 ms reverse and 100 ms readiness dwell. Tests use shortened configurable timings. Continuous updates are required: commands can persist between calls, and a late call can only fault when it arrives. Independent watchdog/actuator dynamics remain required for physical integration.

## Verification and next connection

The Java unit tests inject complete, delayed, unknown, low-RPM, missing-exit, jammed, repeated-sequence and moving-acquisition observations. They cover the two-release mission, serialized commands, one retry, bounded reverse and inhibited planning. Run the standard Gradle test task with `--tests org.agentfrc.lesson.CrescendoMissionSupervisorTest`; no new dependencies are required. Existing original simulation regressions should be rerun separately.

Next: connect this interface to actual acquisition/index/stow plant observations and pose-goal identity, then test feedback-generated mission timing and failure traces. Do not substitute this injected-input success for closed-loop pickup or validate the unchanged shot model by association.

## Separate loaded RPM floor

Config now permits an explicit releaseMinRpm independent of readyRpm. The compatibility constructor preserves the original equal threshold. See A2-SHOOTER-ADAPTER.md for measured same-plant startup/release behavior and the fixed-duty adapter limits; no real sensor data or calibration is implied.
