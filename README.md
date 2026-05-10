# 🚗 Drifter3D

A real-time 3D driving game built from scratch in **C++ and OpenGL 4.5**, featuring a procedurally generated infinite road, dynamic scenery, shadow mapping, and transparent fx.

---

## 🎥 Demo

https://github.com/user-attachments/assets/b67b5c4b-3061-4065-bcb1-660974a6fb05
---

## ✨ Features

### 🛣️ Procedural Road Generation
- Infinite road generated on the fly using a **heading-delta system** with configurable burst curves
- **Catmull-Rom spline** interpolation for smooth road surface quads and barrier lines
- Road reuses a rolling segment pool — old segments ahead of the car are culled and new ones appended behind

### 🌲 Environment
- **Three procedural tree types** — pine, oak, and bush — each built from layered boxes with per-instance colour variation
- **Procedural buildings** spawned on both road sides with randomised heights and colours, including rooftop antenna details
- Trees and buildings are road-aligned and checked for overlap before placement

### ☁️ Transparent Clouds
- Pool of 20 clouds made from multiple overlapping box puffs with soft alpha blending
- Clouds drift laterally and are recycled around the player
- Rendered in a separate **back-to-front transparent pass** with `glDepthMask(GL_FALSE)`

### 🌞 Shadow Mapping
- **4096×4096 depth map** rendered from an orthographic sun light that follows the car
- **PCF 3×3 kernel** for soft shadow edges
- Slope-based bias to eliminate shadow acne
- Shadows cast by the car, buildings, and trees onto the ground

### 🚘 Car Physics
- Arcade-style physics with separate forward/lateral velocity components
- **Tyre grip** model — lateral velocity is damped each frame to simulate understeer
- Speed-scaled steering so turning rate is proportional to current speed
- Flash and bounce-back on collision

### 🎮 Gameplay
- **Cones** (obstacle1.obj) to avoid — collision triggers a crash with a screen flash and life loss
- **Coins** (obstacle2.obj) to collect for score
- 3 lives before game over
- High score tracked per session
- Off-road detection resets the game immediately

### 🖥️ Rendering Pipeline
- Two-pass render: **shadow pass** → **main lit pass**
- `uAlpha` uniform throughout — all geometry shares one shader with optional transparency
- Blended transparent objects (clouds, car windscreen) rendered after all opaques

---

## 🗂️ Project Structure

```
drifter3D/
├── include/                  # Third-party headers (GL, GLFW, GLM, KHR)
│   ├── GL/
│   ├── GLFW/
│   ├── glm/
│   ├── shader.h
│   ├── shadow.h
│   └── ...
├── Lab5b/
│   └── Lab5b/
│       ├── assets/
│       │   ├── obstacle1.obj     # Traffic cone mesh
│       │   └── obstacle2.obj     # Coin mesh
│       ├── main.cpp
│       ├── OBJloader.h
│       ├── shadow.vert           # Depth-only shadow pass vertex shader
│       ├── shadow.frag           # (empty output — depth only)
│       ├── triangle.vert         # Main vertex shader
│       └── triangle.frag         # Main fragment shader (PCF shadows + alpha)
├── lib/                      # Compiled libraries (GLFW, gl3w, etc.)
├── main.sln
└── README.md
```

---

## 🔧 Building

### Requirements
- **Windows** with Visual Studio 2019/2022 (the `.sln` is VS-based)
- OpenGL 4.5-capable GPU and driver
- Dependencies are bundled under `include/` and `lib/` — no separate installs needed

### Steps
1. Clone the repo:
   ```bash
   git clone https://github.com/YOUR_USERNAME/drifter3D.git
   cd drifter3D
   ```
2. Open `main.sln` in Visual Studio
3. Set configuration to **x64 / Debug** (or Release)
4. Build → Run (`F5`)

> Make sure the working directory is set to `Lab5b/Lab5b/` so the shaders and `assets/` folder are found at runtime. In VS: *Project → Properties → Debugging → Working Directory* → `$(ProjectDir)`

---

## 🎮 Controls

| Key | Action |
|-----|--------|
| `W` | Accelerate |
| `S` | Brake / Reverse |
| `A` | Steer left |
| `D` | Steer right |

Drive off the road or lose all 3 lives → game resets.

---

## 🔦 Shader Overview

### `triangle.vert`
Transforms vertices into world space, passes normals (correct inverse-transpose), and outputs `vFragPosLightSpace` for shadow lookup.

### `triangle.frag`
- Selects base colour from `uColor` or `uMaterialKd` (MTL diffuse)
- Computes **diffuse lighting** against `uSunDir`
- Samples `uShadowMap` with a **PCF 3×3** kernel and slope-based bias
- Multiplies result by `uAlpha` for transparency support

### `shadow.vert` / `shadow.frag`
Minimal depth-only pass — renders scene geometry from the light's perspective to populate the shadow map.

---

## ⚙️ Key Constants (tweak in `main.cpp`)

| Constant | Default | Effect |
|----------|---------|--------|
| `ROAD_WIDTH` | `9.0f` | Width of the drivable road |
| `VISIBLE_SEGS` | `130` | Road segments kept in memory |
| `MAX_CURVE_NUDGE` | `0.006f` | Gentle background curve rate |
| `CURVE_BURST_MAG` | `0.018f` | Sharp turn burst magnitude |
| `CLOUD_ALPHA` | `0.55f` | Cloud translucency |
| `SHADOW_ORTHO` | `80.0f` | Shadow frustum half-width |
| `SHADOW_W/H` | `4096` | Shadow map resolution |
| `OBS_POOL` | `20` | Max obstacles on road |
| `maxSpeed` | `0.55f` | Car top speed |

---

## 📦 Dependencies

All bundled — no package manager needed.

| Library | Use |
|---------|-----|
| [GLFW](https://www.glfw.org/) | Window & input |
| [gl3w](https://github.com/skaslev/gl3w) | OpenGL function loader |
| [GLM](https://github.com/g-truc/glm) | Math (vectors, matrices) |
| [stb_image](https://github.com/nothings/stb) | Image loading (header included) |

---

## 📄 License

MIT — free to use, modify, and redistribute. Attribution appreciated.
