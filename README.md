# SuperSlicer 2.7.61.10 EXTRA

My unofficial, personal modification of [SuperSlicer](https://github.com/supermerill/SuperSlicer), based on version 2.7.61.10.

This is **not an official SuperSlicer release**. I made it because I wanted more control over how infill, bridges and perimeters connect, and then added other experimental features that I wanted to test.

I'm not a professional programmer. I developed this version by testing ideas and working with AI coding assistants. AI also helped write this README — I'm lazy. 🙂

The settings are mainly for advanced users who understand what they want to change and why. If you're new to slicing, start with the normal SuperSlicer settings and change one thing at a time.

**To see the extra settings, switch SuperSlicer to Expert mode.**

## My test setup

- Printer: Creality CR-20 Pro
- Controller: 8-bit board
- Firmware: Klipper
- Hotend/nozzle: 0.8 mm Volcano
- Main test material: PETG

My printing profiles if someone is interested are in SS-2.7.61.10-Extra/
├── profiles/
│   ├── printer.ini
│   ├── filament.ini
│   └── print.ini

My prints have been coming out well on this setup, but I have not tested every feature on different printers, materials or firmware.

## What's added?

### 1. Solid-infill-only rotation

Adds a separate setting to rotate solid infill between layers. This is independent of the sparse-infill angle increment.

I use it when experimenting with stronger bonding and watertight parts. Results still depend on line contact, flow, material, geometry and the rest of the print settings.

### 2. Separate overlap controls

Adds independent controls for how these extrusion types connect to perimeters:

- Solid infill → perimeter
- Sparse infill → perimeter
- Bridge infill → perimeter
- Top solid infill → perimeter

This lets you tune each connection separately instead of relying on one shared value. The best settings depend on nozzle diameter, line width, material and model geometry.

Existing line-to-line overlap settings remain separate.

### 3. Sparse-infill overlap flow and speed

Adds separate flow and speed controls for sparse-infill overlap regions. This lets me experiment with slower printing or adjusted flow where infill lines meet other extrusion.

I also experiment with infill connection lengths above 1 mm; my setting is 3 mm. These are experimental settings, not universal recommendations.

### 4. Skip tiny paths

Adds controls to skip very short perimeter and infill paths.

My profile uses a 2 mm threshold for tiny perimeters and diagonal infill paths. The aim is to reduce unwanted short movements and small extrusion fragments. Skipping paths can also remove useful detail, so inspect the sliced preview.

### 5. Pressure-advance map

Adds a pressure-advance map based on speed or volumetric flow, with configurable values and G-code output.

My Klipper profile uses this command format:

```gcode
SET_PRESSURE_ADVANCE ADVANCE={value}
```

For Marlin, `M900 K={value}` may be appropriate if Linear Advance is supported and enabled. The command must match the firmware and its configuration.

Calibrate the map for your own printer, extruder, filament and nozzle.

### 6. Extrusion-multiplier map

Adds the ability to vary extrusion multiplier according to speed or volumetric flow.

The map can be configured in printer settings under Infill and in filament settings under Flow. A filament-specific map overrides the printer-level flow map when enabled.

This is intended to compensate for changes in extrusion behaviour at different operating conditions. It needs testing and calibration.

### 7. Feature-specific flow tuning

Adds separate flow adjustments for different extrusion roles, such as walls, infill and bridges. This gives more control than using one flow value for every type of extrusion.

### 8. Pressure-equalizer ramps

Adds configurable ramp behaviour, including controls for ramp time, distance, segment length and steps. The goal is to make extrusion-pressure changes more gradual.

The effect depends on the printer, firmware and calibration. These settings are experimental.

### 9. Windows build changes

The source has been built on my Windows setup, and I made some build-related changes while getting it working. This does not guarantee that every compiler or Windows configuration will build successfully.

## Printing techniques I use

These are my own experiments and profile settings, not universal recommendations. Start with a backup of your profile and test changes on a small print.

### Alternating perimeter extrusion

My 0.8 mm nozzle profile uses:

- External perimeter width: **1.00 mm**
- Other perimeter width: **1.05 mm**
- External perimeter odd-layer extrusion change: **1.0178**
- Other perimeter odd-layer extrusion change: **0.9839**

The idea is to vary extrusion slightly between alternating layers to encourage more brick-like bonding. Check the result on your own printer and material.

### Solid-infill spacing and rotation

My profile uses:

- Solid-infill spacing: **0.75 mm**
- Alternate-layer solid-infill spacing: **0.72 mm**

The narrower alternate spacing is intended to improve contact between solid-infill lines and neighbouring layers. The rotation setting is separate from sparse-infill angle increment.

### Top-surface spacing

My top-infill spacing is **0.52 mm** with a 0.8 mm nozzle. On my printer, this produces a smooth, almost ironed-looking top surface without a separate slow ironing pass.

This result may not transfer to another material, nozzle or line width.

### Sparse triangle infill and angle rotation

I use a **20° sparse-infill angle increment** for triangle infill.

My usual target is around **7.5–10% infill**, sometimes up to approximately **18%** when I want more Z-direction strength. These are values from my experiments, not a universal strength formula.

For large parts, I experiment with:

- Layer height: **0.15 mm**
- Sparse-infill spacing: **2 mm**
- Nozzle: **0.8 mm Volcano**
- Printed sparse-infill line width: approximately **1.1–1.2 mm**

With this setup, I find that wider infill lines can bond well across several layers, while lines below about 1.5 mm have given me bonding problems in some cases. That is an observation from my printer, not a general minimum.

The goal is to create more XY connections and support upper solid layers while using less plastic than a denser pattern. The trade-off is that sparse infill may provide less Z-direction strength. Inspect the preview and test before relying on this for structural or leak-resistant parts.

The angle increment controls sparse infill; it does not control the separate solid-infill rotation feature.

### Bridge and overhang settings

My profile includes:

- Bridge flow ratio: **50%**
- Bridge speed: **20 mm/s**
- Bridge line density: **100%**
- Bridge wall overlap: **0.5 mm**
- Overhang speed: **16 mm/s**
- Over-bridge flow ratio: **200%**

I experiment with dense support interfaces and increased flow for the first solid layer above a bridge. The aim is to improve bonding between the bridge, the layer above it, perimeters and sparse infill.

The **200% over-bridge flow is my experimental setting**, not a general recommendation. Bridge flow, cooling, temperature, support contact and the first layer above the bridge all interact. Inspect and test these settings carefully.

### Temperature changes by extrusion role

I use custom **Between extrusion role G-code** to choose a first-layer temperature for overhang and bridge roles, while using the normal print temperature for other roles.

My example uses **220°C** for the listed overhang and bridge roles and **245°C** for normal roles:

```text
{if extrusion_role=~/Overhang perimeter/}
M104 S[first_layer_temperature] ; C first layer OVERHANG 220C
{elsif extrusion_role=~/Internal bridge infill/}
M104 S[first_layer_temperature] ; C first layer INTERNAL BRIDGE 220C
{elsif extrusion_role=~/Bridge infill/}
M104 S[first_layer_temperature] ; C first layer BRIDGE 220C
{else}
M104 S[temperature] ; NORMAL, typically 245C in my profile
{endif}
```

The comments describe my intended temperatures; the actual values come from the configured temperature variables. `M104` changes the target temperature without waiting for the nozzle to reach it, so the nozzle may not cool fully before the bridge or overhang starts.

Check that the temperature variables, extrusion-role names and G-code syntax match your SuperSlicer version and printer setup.

### Pressure advance and pressure equalization

My profile combines a pressure-advance map with pressure-equalizer ramp settings to experiment with extrusion consistency as flow changes.

Do not copy these values blindly. Results depend on the extruder, filament, nozzle, temperature, speed and firmware.

## Important profile notes

My current configuration includes:

- Layer height: **0.15 mm**
- First-layer height: **0.25 mm**
- Maximum volumetric speed: **15 mm³/s**
- Perimeters: **3**
- Perimeter generator: **Arachne**
- Travel speed: **400 mm/s**
- Sparse-infill angle increment: **20°** in my triangle-infill experiments

**Check the actual infill density before printing.** One configuration I inspected contains `fill_density = 0%`. That does not set a normal nonzero sparse-infill density. If you expect 7.5–10% infill, verify the effective setting for the object, modifiers and active profile, and confirm the infill in the preview.

This profile is built around a 0.8 mm nozzle. With another nozzle size, review line widths, spacing, flow, overlap, volumetric limits, bridge settings and pressure advance. Values in millimetres and percentages do not scale in the same way.

## Building from source

See the upstream build instructions:

- [Windows build guide](https://github.com/supermerill/SuperSlicer/blob/master/doc/How%20to%20build%20-%20Windows.md)
- [Linux build guide](https://github.com/supermerill/SuperSlicer/blob/master/doc/How%20to%20build%20-%20Linux%20et%20al.md)
- [macOS build guide](https://github.com/supermerill/SuperSlicer/blob/master/doc/How%20to%20build%20-%20Mac%20OS.md)

## Testing and safety

- This is a personal, unofficial build.
- The main test setup is a CR-20 Pro with Klipper and a 0.8 mm Volcano nozzle.
- Not every feature has been independently tested on other hardware or firmware.
- Some controls are experimental and may need calibration.
- Preview the sliced model and test a small print before relying on new settings.
- Back up your working profiles before trying this build.

## Credits and license

Original project: [SuperSlicer](https://github.com/supermerill/SuperSlicer).

SuperSlicer is based on [PrusaSlicer](https://github.com/prusa3d/PrusaSlicer), which is based on [Slic3r](https://github.com/Slic3r/Slic3r).

This is an independently modified fork, not an official upstream release. See the repository's license and upstream project for the applicable license terms and attribution.
