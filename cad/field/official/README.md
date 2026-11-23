# Official field parts

Solids taken unchanged from FIRST's official 2024 field model. The owner approved committing official field models.

| Item | Value |
| --- | --- |
| Source | <https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/CRESCENDO2024Field-STEP.zip>, from FIRST's 2024 Playing Field page |
| Downloaded | 2026-11-23. Server date of the file: 9 April 2024 |
| Zip SHA-256 | `87185d3bf324bb3f35787ed6be715caf6151f6a9e81794830b3f743c381b4f18` (18,652,021 bytes) |
| STEP inside | `FE-2024.STEP`, SHA-256 `42c91b3efb9521d059c13cf25e2d7dda6315bdd6b7eb1202847bfe84ffb43c1a` (88,183,459 bytes; SolidWorks 2022 export dated 2024-04-09) |
| Taken | `speaker-blue.step`: 43 solids of assembly `GE-24300_Briweld Field Blue`, the blue SPEAKER |

The full download is not in the repository. Download it again from the source to check a solid.

## What was taken

The opening, its lip and front panels, the hood, the inner walls a NOTE meets just past the opening, and the subwoofer face and top. Fasteners, rivets and the NOTE counter interior (`GE-24350`) were left out, and so were the body rails and side panels below the opening.

| Solids in `speaker-blue.step` | Official part, under `GE-24300_Briweld Field Blue` |
| --- | --- |
| Speaker opening lip | `GE-24325/GE-24305` |
| Speaker front panel | `GE-24322` |
| Speaker lip panel | `GE-24394` |
| Speaker hood front bar, front panel, top strip | `GE-24310/GE-24319`, `GE-24310/GE-24313`, `GE-24310/GE-24315` |
| Speaker hood top front, top panel, top bar 1 and 2 | `GE-24310/GE-24326/` `GE-24314`, `GE-24317`, `GE-24318` |
| Speaker hood back panel, back bar 1 and 2 | `GE-24310/GE-24324/` `GE-24316`, `GE-24318` |
| Speaker hood side sheet 1 to 4 | `GE-24310/GE-24323/GE-24312` |
| Speaker hood side base, rear, front, nose, top, upper (1 and 2 of each) | `GE-24310/GE-24323/GE-24311/` `-01/-07`, `-02/-08`, `-03`, `-04/-09`, `-05`, `-06/-10` |
| Speaker body rail 5 and 6 | `GE-24301/GE-24301-01`, the two at the opening's height |
| Speaker side panel 3 and 4 | `GE-24303`, the two at the opening's height |
| Speaker inner side wall 1 and 2, inner back wall, inner divider 1 and 2 | `GE-24307`, `GE-24308`, `GE-24309` |
| Subwoofer face 1 to 3, subwoofer top 1 and 2 | `GE-24330_Blue/` `GE-24335`, `GE-24333_Blue` |

Numbered names count repeated parts in order of their bounding boxes.

## Frame

Each solid is where the official file puts it, in that file's frame: millimetres, origin at the field centre, +Y up, and the blue alliance wall at +X. The design source places the whole set with one rigid move into the robot frame and states it there. Field coordinates in the usual blue-origin frame are `x = 8270.5 - X`, `y = Z + 4105.5`, `z = Y` (mm). As a check, the blue SPEAKER's AprilTag 7 plate in the same file lands within 0.2 mm of its published x and y and 4.6 mm of its published height.
